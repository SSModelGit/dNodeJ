using dNodeArch
using SCRIBE: SCRIBEObserverBehavior, SCRIBEObserverState, scribe_observations

struct DataObserverBehavior <: SCRIBEObserverBehavior
    v_s::Dict{Symbol, Float64} # Scalar AWGN process impacting z - stored in dict
    srv_name::String
    agent_id::Integer

    DataObserverBehavior(σₛ::Float64=0.1, srv_name::String="/world_srv", agent_id::Integer=1) = new(Dict(:μ=>0,:σ=>σₛ), agent_id)
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

function scribe_observations(X::Matrix{Float64}, o_b::DataObserverBehavior)
    let nₛ=size(X,1), v_s=o_b.v_s, R=v_s[:σ]*I(nₛ)
        v=Dict(:R=>R, :k=>rand(Gaussian(zeros(nₛ), R)))
        DataObserverState(k, nₛ, X, v, z)
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