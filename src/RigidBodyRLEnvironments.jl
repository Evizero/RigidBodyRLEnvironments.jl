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

include("utils.jl")
include("cartpole/cartpole_sim.jl")
include("cartpole/cartpole_env.jl")
include("cartpole/cartpole_recipe.jl")

end # module
