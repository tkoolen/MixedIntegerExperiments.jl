import BoxRobots

immutable NullBoxRobotController <: BoxRobots.BoxRobotController end
immutable NullBoxRobotControllerData <: BoxRobots.BoxRobotControllerData{NullBoxRobotController} end

function BoxRobots.BoxRobotSimulationData(t::Float64, state::BoxRobots.BoxRobotState, input::BoxRobots.BoxRobotInput)
    BoxRobots.BoxRobotSimulationData(t, state, input, NullBoxRobotControllerData())
end

function Base.convert(::Type{BoxRobots.LimbState}, state::ContactPointState)
    pos = Vector(state.pos)
    vel = zeros(pos)
    in_contact = true # TODO
    BoxRobots.LimbState(pos, vel, in_contact)
end

function Base.convert(::Type{BoxRobots.BoxRobotState}, state::BoxRobotWithRotation2DState)
    comstate = BoxRobots.CentroidalDynamicsState(Vector(state.r), Vector(state.ṙ))
    limbstates = Dict(key => BoxRobots.LimbState(val) for (key, val) in state.contact_point_states)
    BoxRobots.BoxRobotState(comstate, limbstates)
end

function Base.convert(::Type{BoxRobots.LimbInput}, input::ContactPointInput)
    BoxRobots.LimbInput(Vector(input.vel), Vector(input.force), any(x -> x != zero(x), input.force), BoxRobots.ConstantVelocityLimbInput)
end

function Base.convert{B<:BoxRobots.BoxRobotInput, C<:ContactPointInput}(::Type{B}, inputs::Dict{Symbol, C})
    BoxRobots.BoxRobotInput(Dict(key => BoxRobots.LimbInput(val) for (key, val) in inputs))
end

# function Base.convert{S<:BoxRobots.Surface, N}(::Type{S}, region::EnvironmentRegion{N, Float64})
#     hrep = region.position # TODO
#     BoxRobots.Surface{N}(hrep)
# end
#
# function Base.convert{E<:BoxRobots.Environment, N}(::Type{E}, environment::Vector{EnvironmentRegion{N, Float64}})
#     surfaces = map(x -> convert(BoxRobots.Surface{N}, x), environment)
#     BoxRobots.Environment(surfaces)
# end
