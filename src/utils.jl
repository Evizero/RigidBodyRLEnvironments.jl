# We actually don't need the RigidBody simulation record.
struct ThrowAwaySink <: OdeIntegrators.OdeResultsSink
end
OdeIntegrators.initialize(::ThrowAwaySink, args...) = nothing
OdeIntegrators.process(::ThrowAwaySink, args...) = nothing

maybe_rand(rng, x) = x
maybe_rand(rng, x::AbstractArray) = rand(rng, x)
