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

function init_r2_agent(agent_id::Integer=1)
    let r2 = connect_ros2_system()
        init_ros(r2)
        R2SCRIBEAgent(r2, make_node("agent"*string(agent_id), r2), nothing)
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
    [get_observation(init_loc+rand(1,2), r2a, observer, world) for _ in 1:num_samplesᵢ]
    init_agent_loc = world.path[observer.obs_info[:k]]

    # Initialize estimation system and dist learning agent
    lg_Fs = initialize_KF(ag_params, observer, copy(init_agent_loc), world)

    return world, initialize_agent(lg_Fs) 
end

function dist_exploration(agent_id=1; tₘ=1000)
    agent_node = init_r2_agent(agent_id)
    try
        world, agent_node.sagent = dist_agent_setup(agent_id, [0. 0.], 5, agent_node)
        return world, agent_node
    finally
        close_ros(agent_node.r2)
    end
end

function test_subscribing()
    agent_node = init_r2_agent(1)
    subber = add_subscriber(agent_node.r2n, "/chatter", x->println(retrieve_std_string_msg(x)), agent_node.r2.std_msgs["String"], 1)
    try
        while agent_node.r2.rclpy.ok()
            agent_node.r2.rclpy.spin_once(agent_node.r2n.node)
        end
    finally
        close_ros(agent_node.r2)
    end
end