__precompile__()
module RigidBodyRLEnvironments

using ReinforcementLearning
import ReinforcementLearning: step!, reset!, Environment

using POMDPs
using RigidBodyDynamics
using RigidBodyDynamics.OdeIntegrators
using StaticArrays
using RecipesBase

export

    CartpoleSimulator,
    CartpoleEnvironment,
    reset!,
    step!

abstract type RigidBodySimulator end

step!(sim::RigidBodySimulator, args...) =
    step!(Base.GLOBAL_RNG, sim, args...)
reset!(sim::RigidBodySimulator, args...) =
    reset!(Base.GLOBAL_RNG, sim, args...)

include("utils.jl")
include("cartpole/cartpole_sim.jl")
include("cartpole/cartpole_env.jl")
include("cartpole/cartpole_recipe.jl")

end # module
