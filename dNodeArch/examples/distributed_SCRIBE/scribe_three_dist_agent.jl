using dNodeArch
using SCRIBE: SCRIBEAgent, SCRIBEModel, SCRIBEObserverBehavior, SCRIBEObserverState, LGSFModelParameters, LGSFModel, initialize_KF, initialize_agent
import SCRIBE: scribe_observations, get_model_time

using Parameters: @unpack
using LinearAlgebra: I
using GaussianDistributions: Gaussian

mutable struct R2SCRIBEAgent
    r2::R2Artifacts
    r2n::R2Node
    sagent::Union{SCRIBEAgent, Nothing}

    R2SCRIBEAgent(r2::R2Artifacts, r2n::R2Node, sagent::Union{SCRIBEAgent, Nothing}) = new(r2, r2n, sagent)
end

function init_r2_agent(agent_tag::String="agent1")
    let r2 = connect_ros2_system()
        init_ros(r2)
        R2SCRIBEAgent(r2, make_node(agent_tag, r2), nothing)
    end
end

struct DataServerWorld <: SCRIBEModel
    srv_info::R2ServiceInfo
    path::Dict{Integer, Matrix{Float64}}
    true_obs::Dict{Integer, Array{Any}}

    DataServerWorld(srv_info::R2ServiceInfo, path::Dict, true_obs::Dict) = new(srv_info, path, true_obs)
end

connect_world_server(srv_info::R2ServiceInfo, path_dims::Integer) = DataServerWorld(srv_info, Dict(1=>Matrix{Float64}(undef, 0, path_dims)), Dict(1=>Any[]))

"""For now, just pick arbitrary time.
"""
get_model_time(smodel::DataServerWorld) = 10

struct DataObserverBehavior <: SCRIBEObserverBehavior
    v_s::Dict{Symbol, Float64} # Scalar AWGN process impacting z - stored in dict
    obs_info::Dict{Symbol, Integer}

    DataObserverBehavior(v_s::Dict, obs_info::Dict) = new(v_s, obs_info)
end

data_observer(σₛ::Float64=0.1, agent_id::Integer=1) = DataObserverBehavior(Dict(:μ=>0,:σ=>σₛ), Dict(:id=>agent_id, :k=>1))

"""Not thread-safe!
"""
function get_observation(loc::Matrix{Float64}, agent::R2SCRIBEAgent, observer::DataObserverBehavior, world::DataServerWorld; timeout::Float64=1.0)
    @unpack r2, r2n = agent
    @unpack srv_info, path, true_obs = world
    k = observer.obs_info[:k]
    data = Dict([("x"*string(i), x) for (i,x) in enumerate(loc)])
    let response = request_r2_service(r2n, srv_info, data, timeout, r2)
        if response[1] == 200
            path[k] = vcat(path[k], loc)
            push!(true_obs[k], float(response[2]["state"]))
            return 200
        end
        println("World unobservable...")
        return 500
    end
end

"""Ensures that the matrix index slice is also of type Matrix.
"""
mat_row(w::Integer, X::Matrix) = reshape(X[w, :], 1, size(X, 2))

"""Convenience function to loop through all locations specified in X.
"""
function get_n_observations(;X::Matrix{Float64}, agent::R2SCRIBEAgent, observer::DataObserverBehavior, world::DataServerWorld, timeout::Float64=1.0)
    for i in 1:size(X,1)
        get_observation(mat_row(i,X), agent, observer, world; timeout=timeout)
    end
end

struct DataObserverState <: SCRIBEObserverState
    k::Integer
    nₛ::Integer
    X::Matrix{Float64}
    v::Dict{Symbol, AbstractArray{Float64}}
    z::Vector{Float64}

    function DataObserverState(k::Integer, nₛ::Integer,
                               X::Matrix{Float64}, v::Dict{Symbol, AbstractArray{Float64}},
                               z::Vector{Float64})
        new(k, nₛ, X, v, z)
    end
end

"""Not thread-safe!
"""
function scribe_observations(X::Matrix{Float64}, smodel::DataServerWorld, o_b::DataObserverBehavior)
    let nₛ=size(X,1), k=o_b.obs_info[:k]
        R=o_b.v_s[:σ]*I(nₛ)
        v=Dict(:R=>R, :k=>rand(Gaussian(zeros(nₛ), R)))
        z = smodel.true_obs[k] + v[:k]
        # k, sensor_in = [[p[i] for p in o_b.path] for i in 1:3]
        # z = sensor_in + v[:k]
        DataObserverState(k, nₛ, X, v, z)
    end
end

function dist_agent_setup(id::Integer, init_loc::Matrix{Float64}, num_samplesᵢ::Integer, r2a::R2SCRIBEAgent; nᵩ=2)
    # Initialize ROS2 distributed network system
    @unpack r2, r2n = r2a
    srv_info = connect_ground_model("two_dim_world_state_req", "TwoDimWorldScalarStateReq", r2)
    world = connect_world_server(srv_info, 2)

    # Initialize parameters for environment learning
    ag_params=LGSFModelParameters(μ=hcat(range(-1,1,nᵩ), zeros(nᵩ)),σ=[1.],τ=[1.],
                                  ϕ₀=zeros(nᵩ), A=Matrix{Float64}(I(nᵩ)),
                                  Q=0.0001*Matrix{Float64}(I(nᵩ)))
    observer = data_observer(0.1, id)
    init_agent_loc=vcat(zeros(nᵩ)', init_loc)

    # Gather observations along random walk to provide early prior
    init_agent_loc = vcat(init_loc, init_loc.+rand(num_samplesᵢ-1, size(init_loc,2)))
    get_n_observations(; X=init_agent_loc, agent=r2a, observer=observer, world=world, timeout=1.0)
    # [get_observation(init_loc+rand(1,2), r2a, observer, world) for _ in 1:num_samplesᵢ]
    # init_agent_loc = world.path[observer.obs_info[:k]]

    # Initialize estimation system and dist learning agent
    lg_Fs = initialize_KF(ag_params, observer, copy(init_agent_loc), world)

    return world, initialize_agent(lg_Fs) 
end

"""Callback function for updating an agent's network map.
"""
function update_network_map(r2data::Any, ag_tag::String, netmap::Dict)
    let new_map=retrieve_std_string_json(r2data)
        for k in keys(netmap)
            netmap[k] = new_map[ag_tag][k]
        end
    end
end

"""General callback to store updates from other agents.
"""
store_update(r2data::Any, storage::Dict, ext_ag::String) = push!(storage[ext_ag], retrieve_std_string_json(r2data))

"""Helper function to add all necessary subscribers for a given agent.
"""
function listen_to_agent(ext_ag::String, covi_store::Dict, mhmc_store::Dict, anode::R2SCRIBEAgent)
    @unpack r2, r2n = anode
    add_subscriber(r2n, "/"*ext_ag*"/covi", x->store_update(x, covi_store, ext_ag), r2.std_msgs.String, 1)
    add_subscriber(r2n, "/"*ext_ag*"/mhmc", x->store_update(x, mhmc_store, ext_ag), r2.std_msgs.String, 1)
end

"""Helper function to destroy all subscribers associated with a given agent.
"""
function stop_listening_to_agent(anode::R2SCRIBEAgent, ext_ag::String)
    @unpack r2n = anode
    remove_subscriber(r2n, "/"*ext_ag*"/covi")
    remove_subscriber(r2n, "/"*ext_ag*"/mhmc")
end

function wpt_following_observer(;Xₖ::Matrix{Float64}, wpt::Matrix{Float64}, nₛ::Integer,
                                 agent::R2SCRIBEAgent, observer::DataObserverBehavior, world::DataServerWorld, timeout::Float64=1.0)
    get_n_observations(;X=hcat(range(start=Xₖ[1],stop=wpt[1],length=nₛ), range(start=Xₖ[2],stop=wpt[2],length=nₛ)),
                        agent=agent, observer=observer, world=world, timeout=timeout)
end

function dist_exploration(agent_id=1;nₐ=3, tₘ=1000; param_file="agent_params.json")
    agent_tag = "agent"*string(agent_id)
    agent_params = parse(read(param_file, String))
    wpts = agent_params[agent_tag]["locs"]["wpts"]
    X₀=agent_params[agent_tag]["locs"]["x0"]
    agent_node = init_r2_agent(agent_tag)
    agent_connections = Dict{String, Bool}([("agent"*string(i), false) for i in 1:nₐ if "agent"*string(i)≠agent_tag])

    # Consensus data for Cov. Int. and MHMC
    covi_cdata = Dict(String, Array)([(ext_tag, Dict[]) for ext_tag in keys(agent_connections)])
    mhmc_cdata = Dict(String, Array)([(ext_tag, Dict[]) for ext_tag in keys(agent_connections)])

    try
        world, agent_node.sagent = dist_agent_setup(agent_id, [0. 0.], 5, agent_node)

        # Start listening to Network Mapper
        add_subscriber(agent_node.r2n, "/network_map", x->update_network_map(x, agent_tag, agent_connections), agent_node.r2.std_msgs.String, 1)
        agent_node.r2.rclpy.spin_once(agent_node.r2n.node) # Get the first network map upates

        # Add agent publishers
        add_publisher(agent_node.r2n, "/"*agent_tag*"/covi", agent_node.r2.std_msgs.String, 10)
        add_publisher(agent_node.r2n, "/"*agent_tag*"/mhmc", agent_node.r2.std_msgs.String, 10)

        while true
            # Add subscribers to other agents based on connection networks
            for (ext_ag, connectedQ) in agent_connections;
                if connectedQ listen_to_agent(ext_ag, covi_cdata, mhmc_cdata, agent_node) end
            end

            return world, agent_node
        end
    finally
        close_ros(agent_node.r2)
    end
end

if length(ARGS) < 1
    println("No agent ID provided!")
else
    agent_id = ARGS[1]
    println("Agent ID: ", agent_id)
end

# function test_subscribing()
#     agent_node = init_r2_agent(1)
#     subber = add_subscriber(agent_node.r2n, "/chatter", x->println(retrieve_std_string_json(x)), agent_node.r2.std_msgs["String"], 1)
#     try
#         while agent_node.r2.rclpy.ok()
#             agent_node.r2.rclpy.spin_once(agent_node.r2n.node)
#         end
#     finally
#         close_ros(agent_node.r2)
#     end
# end