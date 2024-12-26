using dNodeArch: connect_ros2_system, init_ros, make_node, add_publisher, add_subscriber, connect_ground_model, publish_std_string_msg, retrieve_std_string_msg, request_r2_service
using SCRIBE: SCRIBEModel, SCRIBEObserverBehavior, SCRIBEObserverState, scribe_observations, initialize_KF, initialize_agent
using Parameters: @unpack

mutable struct R2SCRIBEAgent
    r2::R2Artifacts
    r2n::R2Node
    sagent::Union{SCRIBEAgent, Nothing}

    R2SCRIBEAgent(r2::R2Artifacts, r2n::R2Node, sagent::SCRIBEAgent) = new(r2, r2n, sagent)
end

struct DataServerWorld <: SCRIBEModel
    srv_info::R2ServiceInfo
    path::Dict{Integer, Matrix{Float64}}
    true_obs::Dict{Integer, Array{Any}}

    DataServerWorld(srv_info::R2ServiceInfo, path::Dict{Integer, Matrix{Float64}}, true_obs::Dict{Integer, Array{Any}}) = new(srv_info, path, true_obs)
end

connect_world_server(srv_info::R2ServiceInfo, path_dims::Integer) = DataServerWorld(srv_info, Matrix{Float64}(undef, 0, path_dims), Any[])

"""For now, just pick arbitrary time.
"""
get_model_time(smodel::DataServerWorld) = 10

struct DataObserverBehavior <: SCRIBEObserverBehavior
    v_s::Dict{Symbol, Float64} # Scalar AWGN process impacting z - stored in dict
    obs_info::Dict{Symbol, Integer}

    DataObserverBehavior(v_s::Dict{Symbol, Float64}, obs_info::Dict{Symbol, Integer}) = new(v_s, obs_info)
end

data_observer(σₛ::Float64=0.1, agent_id::Integer) = DataObserverBehavior(Dict(:μ=>0,:σ=>σₛ), Dict(:id=>agent_id, :k=>1))

"""Not thread-safe!
"""
function get_observation(loc::Matrix{Float64}, agent::R2SCRIBEAgent, observer::DataObserverBehavior, world::DataServerWorld; timeout::Float64=1.0)
    @unpack r2, r2n = agent
    @unpack srv_info, path, true_obs = world
    k = observer.obs_info[:k]
    data = Dict([("x"*string(i), x) for (i,x) in enumerate(loc)])
    let response = request_r2_service(r2n, srv_info, data, timeout, r2)
        if response[1] == 200
            vcat(path[k], loc)
            push!(true_obs[k], response[2]["state"])
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
        z = smodel.true_obs + v[:k]
        # k, sensor_in = [[p[i] for p in o_b.path] for i in 1:3]
        # z = sensor_in + v[:k]
        DataObserverState(k, nₛ, X, v, z)
    end
end

function dist_agent_setup(id::Integer=1, init_loc::Matrix{Float64}=[-0.5 -0.5], num_samplesᵢ::Integer=5, r2a::R2SCRIBEAgent)
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

    # Initialize estimation system and dist learning agent
    lg_Fs = initialize_KF(ag_params, observer, copy(init_agent_loc), world)

    return world, initialize_agent(lg_Fs) 
end

function dist_exploration(agent_id=1; tₘ=1000)
    r2 = connect_ros2_system()
    init_ros()
    r2node = make_node("agent"*string(agent_id), r2)

    agent_node = R2SCRIBEAgent(r2, r2n, nothing)
    world, agent_node.sagent = dist_agent_setup(agent_id, [0. 0.], 5, agent_node)
end