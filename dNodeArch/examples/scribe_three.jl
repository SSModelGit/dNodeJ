using dNodeArch: request_r2_service
using SCRIBE: SCRIBEObserverBehavior, SCRIBEObserverState, scribe_observations
using Parameters: @unpack

struct DataObserverBehavior <: SCRIBEObserverBehavior
    v_s::Dict{Symbol, Float64} # Scalar AWGN process impacting z - stored in dict
    srv_info::R2ServiceInfo
    agent_id::Integer
    path::Matrix{Float64}
    sensor_input::Array{Any}

    DataObserverBehavior(v_s::Dict{Symbol, Float64}, srv_info::R2ServiceInfo, agent_id::Integer, path::Matrix{Float64}, sensor_input::Array{Any}) = new(v_s, srv_info, agent_id, path, sensor_input)
end

basic_observer(σₛ::Float64=0.1, srv_info::R2ServiceInfo, agent_id::Integer, path_dims::Integer) = DataObserverBehavior(Dict(:μ=>0,:σ=>σₛ), srv_info, agent_id, Matrix{Float64}(undef, 0, path_dims), Any[])

"""Not thread-safe!
"""
function get_observation(k::Integer, loc::Matrix{Float64}, o_b::DataObserverBehavior, r2n::R2Node, timeout::Float64, r2::R2Artifacts)
    @unpack srv_info = o_b
    data = Dict([("x"*string(i), x) for (i,x) in enumerate(loc)])
    let response = request_r2_service(r2n, srv_info, data, timeout, r2)
        if response[1] == 200
            vcat(o_b.path, loc)
            push!(o_b.sensor_input, (k, response[2]["state"]))
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
function scribe_observations(X::Matrix{Float64}, o_b::DataObserverBehavior)
    let nₛ=size(X,1), v_s=o_b.v_s, R=v_s[:σ]*I(nₛ)
        v=Dict(:R=>R, :k=>rand(Gaussian(zeros(nₛ), R)))
        k, sensor_in = [[p[i] for p in o_b.path] for i in 1:3]
        @assert all(k.==mean(k))
        z = sensor_in + v[:k]
        empty!(o_b.path)
        empty!(o_b.sensor_input)
        DataObserverState(mean(k), nₛ, X, v, z)
    end
end

function agent_setup(nᵩ=2, init_loc=[-0.5 -0.5])
    ag_params=LGSFModelParameters(μ=hcat(range(-1,1,nᵩ), zeros(nᵩ)),σ=[1.],τ=[1.],
                                  ϕ₀=zeros(nᵩ), A=Matrix{Float64}(I(nᵩ)),
                                  Q=0.0001*Matrix{Float64}(I(nᵩ)))
    observer=LGSFObserverBehavior(0.01)
    init_agent_loc=vcat(zeros(nᵩ)', init_loc)
    lg_Fs = initialize_KF(ag_params, observer, copy(init_agent_loc), gt_model[1])
    return initialize_agent(lg_Fs)
end