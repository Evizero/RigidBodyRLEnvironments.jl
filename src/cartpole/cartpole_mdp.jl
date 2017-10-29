struct CartpoleMDP{E<:CartpoleEnvironment,F,R,T} <: MDPView{SVector{4,Float64},Float64}
    env::E
    actions::Vector{Float64}
    statefun::F # maps environment state to mdp state
    rewardfun::R # maps envionment state plus action to reward
    isterminalfun::T
end

function cartpole_statefun(mdp::CartpoleMDP, envstate)
    config = configuration(envstate)
    vel = velocity(envstate)
    @inbounds begin
        cart_x = config[1]
        cart_x_dot = vel[1]
        pole_angle = config[2]
        pole_angle_dot = vel[2]
    end
    SVector(cart_x, cart_x_dot, pole_angle, pole_angle_dot)
end

function cartpole_isterminalfun(pos_limit, angle_limit)
    function (mdp::CartpoleMDP, s)
        abs(s[1]) >= pos_limit || abs(s[3]) >= angle_limit
    end
end

# use common default values
function CartpoleMDP(;
        M = 1., m = 0.1, l = 1., com = l/2, dt = 2e-2,
        pos_limit = 2.4, angle_limit = deg2rad(12),
        actions = [-10.,0.,10.],
        statefun = cartpole_statefun,
        rewardfun = (mdp, s, a, envstate) -> 1,
        isterminalfun = cartpole_isterminalfun(pos_limit, angle_limit)
       )
    env = CartpoleEnvironment(M, m, l, com, (-pos_limit, pos_limit), dt)
    CartpoleMDP(env, convert(Vector{Float64}, actions), statefun, rewardfun, isterminalfun)
end

POMDPs.actions(mdp::CartpoleMDP) = mdp.actions
POMDPs.n_actions(mdp::CartpoleMDP) = length(mdp.actions)

function POMDPs.initial_state(mdp::CartpoleMDP, rng::AbstractRNG)
    envstate = resetenv!(rng, mdp.env, 0.1 .* rand(rng,SVector{4}) .- 0.05)
    mdp.statefun(mdp, envstate)
end

function POMDPs.generate_sr(mdp::CartpoleMDP, s, a, rng::AbstractRNG)
    envstate = stepenv!(mdp.env, a)
    mdp.statefun(mdp, envstate), mdp.rewardfun(mdp, s, a, envstate)
end

function POMDPs.isterminal(mdp::CartpoleMDP, s)
    mdp.isterminalfun(mdp, s)
end
