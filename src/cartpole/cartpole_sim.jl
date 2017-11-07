# Note that the cartpole scenario can actually be efficiently
# hardcoded as a proper MDP. This implementation is more of a proof
# of concept on how systems can be specified.

mutable struct CartpoleSimulator{M<:Mechanism,S<:MechanismState,I} <: RigidBodySimulator
    mechansim::M
    state::S
    integrator::I
    torques::Vector{Float64}
    time::Float64
    cart_mass::Float64
    pole_mass::Float64
    pole_length::Float64
    pole_com::Float64 # center of mass
    simstep::Float64 # timestep for advancing the physics simulation
    dt::Float64 # timestep for observing next state (also duration for applying an action)
    xlimits::Tuple{Float64,Float64}
end

function CartpoleSimulator(;
        M=1., m=.1, l=1., com=l, xlim::Tuple=(-2.5,2.5),
        dt::Float64=2e-2, simstep::Float64=2e-2, g=-9.8)
    cart_mass = Float64(M)
    pole_mass = Float64(m)
    pole_length = Float64(l)
    pole_com = Float64(com)
    xlimits = map(Float64, xlim)
    gravity = Float64(g)
    CartpoleSimulator(cart_mass, pole_mass, pole_length, pole_com, xlimits, dt, simstep, gravity)
end

function CartpoleSimulator(
        cart_mass::Float64,
        pole_mass::Float64,
        pole_length::Float64,
        pole_com::Float64,
        xlimits::Tuple{Float64,Float64} = (-2.5,2.5),
        dt::Float64 = 2e-2,
        simstep::Float64 = 2e-2,
        gravity::Float64 = -9.8)
    @assert simstep <= dt
    world = RigidBody{Float64}("world")
    mechanism = Mechanism(world; gravity = SVector(0., 0., gravity))

    # create massless slidebar that will restrict cart to x-axis
    inertia1 = SpatialInertia(CartesianFrame3D("slidebar"), zeros(SMatrix{3,3}), zeros(SVector{3}), 0.)
    slidebar = RigidBody(inertia1)
    slidebar_to_world = Joint("slidebar_to_world", Fixed{Float64}())
    before_slidebar_to_world = eye(Transform3D, frame_before(slidebar_to_world), default_frame(world))
    attach!(mechanism, world, slidebar, slidebar_to_world, joint_pose = before_slidebar_to_world)

    # create cart of mass M and connect to slidebar
    inertia2 = SpatialInertia(CartesianFrame3D("cart"), zeros(SMatrix{3,3}), zeros(SVector{3}), cart_mass)
    cart = RigidBody(inertia2)
    cart_to_slidebar = Joint("cart_to_slidebar", Prismatic(SVector(1.,0.,0.)))
    before_cart_to_slidebar = Transform3D(frame_before(cart_to_slidebar), frame_after(slidebar_to_world), zeros(SVector{3}))
    attach!(mechanism, slidebar, cart, cart_to_slidebar, joint_pose = before_cart_to_slidebar)
    # set some joint limits (currently not enforced I think)
    cart_to_slidebar.effort_bounds[1] = RigidBodyDynamics.Bounds{Float64}(-1e3,1e3)
    cart_to_slidebar.position_bounds[1] = RigidBodyDynamics.Bounds{Float64}(xlimits...)
    cart_to_slidebar.velocity_bounds[1] = RigidBodyDynamics.Bounds{Float64}(-5,5)

    # create pole of length l with center of mass in com
    inertia3 = SpatialInertia(CartesianFrame3D("pole"),
                              @SMatrix([0. 0. 0.; 0. pole_mass*pole_length^2 0.; 0. 0. 0.]),
                              pole_mass * SVector(0., 0., pole_com),
                              pole_mass)
    pole = RigidBody(inertia3)
    pole_to_cart = Joint("pole_to_cart", Revolute(SVector(0.,1.,0.)))
    before_pole_to_cart = eye(Transform3D, frame_before(pole_to_cart), frame_after(cart_to_slidebar))
    attach!(mechanism, cart, pole, pole_to_cart, joint_pose = before_pole_to_cart)

    # Set up simulator
    state = MechanismState(mechanism)
    TE = RigidBodyDynamics.cache_eltype(state)
    result = RigidBodyDynamics.DynamicsResult{TE}(state.mechanism)
    torques = zeros(velocity(state))
    closed_loop_dynamics! = (v̇::AbstractArray, ṡ::AbstractArray, t, state) -> begin
        RigidBodyDynamics.dynamics!(result, state, torques)
        #RigidBodyDynamics.zero_torque!(torques, t, state) # reset after applied
        copy!(v̇, result.v̇)
        copy!(ṡ, result.ṡ)
        nothing
    end
    tableau = RigidBodyDynamics.runge_kutta_4(Float64)
    integrator = RigidBodyDynamics.MuntheKaasIntegrator(closed_loop_dynamics!, tableau, ThrowAwaySink())
    OdeIntegrators.set_num_positions!(integrator.stages, length(configuration(state)))
    OdeIntegrators.set_num_velocities!(integrator.stages, length(velocity(state)))
    OdeIntegrators.set_num_additional_states!(integrator.stages, length(additional_state(state)))
    sim = CartpoleSimulator{typeof(mechanism),typeof(state),typeof(integrator)}(
        mechanism,
        state,
        integrator,
        torques,
        0.,
        cart_mass,
        pole_mass,
        pole_length,
        pole_com,
        simstep,
        dt,
        xlimits
    )
    reset!(sim)
    sim
end

function reset!(rng::AbstractRNG, sim::CartpoleSimulator, s0 = SVector(0.,0.,0.,0.))
    configuration(sim.state)[1] = Float64(s0[1])
    configuration(sim.state)[2] = Float64(s0[3])
    velocity(sim.state)[1] = Float64(s0[2])
    velocity(sim.state)[2] = Float64(s0[4])
    setdirty!(sim.state)
    sim.time = 0.
    sim.torques .= 0.
    sim.state
end

function step!(rng::AbstractRNG, sim::CartpoleSimulator, force::Number = 0.)
    steps = floor(Int, sim.dt / sim.simstep)
    for i in 1:steps
        @inbounds sim.torques[1] = Float64(force)
        OdeIntegrators.step(sim.integrator, sim.time, sim.state, sim.simstep)
        sim.time += sim.simstep
    end
    sim.state
end
