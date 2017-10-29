__precompile__()
module RigidBodyRLEnvironments

using POMDPs
using RigidBodyDynamics
using RigidBodyDynamics.OdeIntegrators
using StaticArrays
using RecipesBase

export

    CartpoleEnvironment,
    CartpoleMDP,
    resetenv!,
    stepenv!

abstract type RigidBodyRLEnvironment end
function resetenv! end # resetenv!([rng], env) -> response0
function stepenv!  end # step!([rng], env, a) -> response

resetenv!(env::RigidBodyRLEnvironment, args...) =
    resetenv!(Base.GLOBAL_RNG, env, args...)
stepenv!(env::RigidBodyRLEnvironment, args...) =
    stepenv!(Base.GLOBAL_RNG, env, args...)

abstract type MDPView{S,A} <: MDP{S,A} end

include("utils.jl")
include("cartpole/cartpole_env.jl")
include("cartpole/cartpole_mdp.jl")
include("cartpole/cartpole_recipe.jl")

end # module
