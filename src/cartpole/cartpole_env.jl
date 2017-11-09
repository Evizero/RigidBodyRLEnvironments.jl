struct CartpoleEnvironment{E<:CartpoleSimulator,RNG,F,R,I,T} <: Environment
    sim::E
    rng::RNG
    actions::Vector{Float64}
    discount::Float64
    statefun::F # maps environment state to env state
    rewardfun::R # maps envionment state plus action to reward
    initfun::I
    isterminalfun::T
end

function cartpole_statefun(env::CartpoleEnvironment, simstate)
    config = configuration(simstate)
    vel = velocity(simstate)
    @inbounds begin
        cart_x = config[1]
        cart_x_dot = vel[1]
        pole_angle = config[2]
        pole_angle_dot = vel[2]
    end
    SVector(cart_x, cart_x_dot, pole_angle, pole_angle_dot)
end

function cartpole_isterminalfun(pos_limit, angle_limit)
    function (env::CartpoleEnvironment, s)
        abs(s[1]) >= pos_limit || abs(s[3]) >= angle_limit
    end
end

# use common default values
function CartpoleEnvironment(;
        M = 1., m = 0.1, l = 1., com = l/2, dt = 2e-2,
        pos_limit = 2.4, angle_limit = deg2rad(12),
        rng = Base.GLOBAL_RNG,
        actions = [-10.,0.,10.],
        discount = 0.99,
        statefun = cartpole_statefun,
        rewardfun = (env, a, simstate) -> 1,
        initfun = (env) -> (0.1 .* rand(env.rng,SVector{4}) .- 0.05),
        isterminalfun = cartpole_isterminalfun(pos_limit, angle_limit)
       )
    sim = CartpoleSimulator(M, m, l, com, (-pos_limit, pos_limit), dt)
    CartpoleEnvironment(sim, rng, convert(Vector{Float64}, actions), Float64(discount), statefun, rewardfun, initfun, isterminalfun)
end

function Base.show(io::IO, env::CartpoleEnvironment)
    println(io, typeof(env).name)
    println(io, "  time: ", env.sim.time, " s")
    print(io, "  state: ", env.statefun(env, env.sim.state))
end

POMDPs.discount(env::CartpoleEnvironment) = env.discount
POMDPs.actions(env::CartpoleEnvironment) = env.actions
POMDPs.action_index(env::CartpoleEnvironment, a) = findfirst(env.actions, a)
POMDPs.n_actions(env::CartpoleEnvironment) = length(env.actions)

function reset!(env::CartpoleEnvironment)
    simstate = reset!(env.sim, env.initfun(env))
    env.statefun(env, simstate)
end

function step!(env::CartpoleEnvironment, a)
    simstate = step!(env.sim, a)
    env.statefun(env, simstate), env.rewardfun(env, a, simstate)
end

function POMDPs.isterminal(env::CartpoleEnvironment)
    env.isterminalfun(env, env.statefun(env, env.sim.state))
end
