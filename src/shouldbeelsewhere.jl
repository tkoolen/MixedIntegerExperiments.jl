export Side, left, right, flipsign_if_right, findbody, create_valkyrie, visualize_environment

@enum Side left right
flipsign_if_right(x::Number, side::Side) = ifelse(side == right, -x, x)

function findbody(mechanism::Mechanism, name::String)
    results = filter(b -> b.name == name, bodies(mechanism))
    isempty(results) && error("no body found with name \"$(name)\"")
    length(results) > 1 && error("multiple bodies found with name \"$(name)\": $results")
    first(results)
end

function findjoint(mechanism::Mechanism, name::String)
    results = filter(j -> j.name == name, joints(mechanism))
    isempty(results) && error("no joint found with name \"$(name)\"")
    length(results) > 1 && error("multiple joints found with name \"$(name)\": $results")
    first(results)
end

using RigidBodyDynamics.Contact
function create_valkyrie(urdf::String)
    # TODO: grab everything off of github and cache
    T = Float64
    mechanism = RigidBodyDynamics.parse_urdf(T, urdf);

    # pelvis
    pelvis = findbody(mechanism, "pelvis")

    # floating joint
    pelvis_to_world = joint_to_parent(pelvis, mechanism)
    pelvis_to_world.jointType = QuaternionFloating{Float64}()

    # extremities
    feet = Dict(side => findbody(mechanism, "$(side)Foot") for side in instances(Side))
    hands = Dict(side => findbody(mechanism, "$(side)Palm") for side in instances(Side))
    # TODO: amputate fingers

    # relevant joints
    hippitches = Dict(side => findjoint(mechanism, "$(side)HipPitch") for side in instances(Side))
    knees = Dict(side => findjoint(mechanism, "$(side)KneePitch") for side in instances(Side))
    anklepitches = Dict(side => findjoint(mechanism, "$(side)AnklePitch") for side in instances(Side))

    # add sole frames
    soleframes = Dict(side => CartesianFrame3D("$(side)Sole") for side in instances(Side))
    for side in instances(Side)
        foot = feet[side]
        soleframe = soleframes[side]
        add_frame!(foot, Transform3D(soleframe, default_frame(foot), SVector(0.067, 0., -0.09)))
    end

    # add foot contact points
    contactmodel = SoftContactModel(hunt_crossley_hertz(k = 100e3), ViscoelasticCoulombModel(0.8, 20e3, 10.))
    for side in instances(Side)
        foot = feet[side]
        frame = default_frame(foot)
        z = -0.09
        add_contact_point!(foot, ContactPoint(Point3D(frame, -0.038, flipsign_if_right(0.55, side), z), contactmodel))
        add_contact_point!(foot, ContactPoint(Point3D(frame, -0.038, flipsign_if_right(-0.55, side), z), contactmodel))
        add_contact_point!(foot, ContactPoint(Point3D(frame, 0.172, flipsign_if_right(0.55, side), z), contactmodel))
        add_contact_point!(foot, ContactPoint(Point3D(frame, 0.172, flipsign_if_right(-0.55, side), z), contactmodel))
    end

    # add environment
    rootframe = root_frame(mechanism)
    floor = HalfSpace3D(Point3D(rootframe, 0., 0., 0.), FreeVector3D(rootframe, 0., 0., 1.))
    leftwall = HalfSpace3D(Point3D(rootframe, 0., -1., 1.5), FreeVector3D(rootframe, 0., 1., 0.))
    rightwall = HalfSpace3D(Point3D(rootframe, 0, 1., 1.5), FreeVector3D(rootframe, 0., -1., 0.))
    add_environment_primitive!(mechanism, floor)
    add_environment_primitive!(mechanism, leftwall)
    add_environment_primitive!(mechanism, rightwall)

    mechanism, feet, hands, pelvis, hippitches, knees, anklepitches, pelvis_to_world
end

using DrakeVisualizer, CoordinateTransformations, FixedSizeArrays, ColorTypes, Rotations
function visualize_environment(mechanism::Mechanism, vis::Visualizer)
    root = root_body(mechanism)
    rootframe = default_frame(root)
    for (i, halfspace) in enumerate(mechanism.environment.halfspaces) # TODO: consider naming them
        name = "half-space $i"
        @framecheck Contact.frame(halfspace) rootframe # TODO
        R = Rotations.rotation_between(SVector(0., 0., 1.), halfspace.outward_normal.v)
        p = halfspace.point.v
        tf = Translation(p) ∘ LinearMap(R)
        thickness = 0.1
        length = 3.
        box = HyperRectangle(Vec(-length/2, -length/2, -thickness), Vec(length, length, thickness))
        box_vis = setgeometry!(vis[:environment][Symbol(name)], GeometryData(box, RGBA(0, 0, 1, 0.5)))
        settransform!(box_vis, tf)
    end
end

# radius = 0.01
# for foot in values(feet)
#     spheres = [HyperSphere(Point(location(point).v...), radius) for point in contact_points(foot)]
#     geomdata = [GeometryData(sphere, RGBA(1, 0, 0, 0.5)) for sphere in spheres]
#     dict = Dict{CartesianFrame3D, Vector{GeometryData}}(default_frame(foot) => geomdata)
#     setgeometry!(vis[:contactpoints][Symbol(name(foot))], dict)
# end
