export visualize_environment

using RigidBodyDynamics
using DrakeVisualizer, CoordinateTransformations, FixedSizeArrays, ColorTypes, Rotations
using RigidBodyDynamics.Contact

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
