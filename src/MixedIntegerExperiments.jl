module MixedIntegerExperiments

using JuMP
using Gurobi
using MultilinearOpt
using Polyhedra, CDDLib
using Plots
using DrakeVisualizer
using ColorTypes
using AxisArrayVariables
using AxisArrays
using StaticArrays
using Parameters

include("util.jl")
include("region_descriptions.jl")
include("disjoint_union.jl")
include("mccormick.jl")
include("plotting.jl")
include("shouldbeelsewhere.jl")
include("trajopt.jl")

export
    RegionDescription,
    EnvironmentRegion,
    MIQPTrajOptParams,
    ContactPointDescription,
    BoxRobotWithRotation2D,
    ContactPointState,
    BoxRobotWithRotation2DState,
    ContactPointInput,
    axis_aligned_bounding_box,
    contact_region,
    isfree,
    axis_aligned_free_box_region,
    plot_polyhedron!,
    plot_environment,
    plot_allowable_forces,
    plot_kinematic_regions,
    miqp_trajopt,
    rms_constraint_violation

end # module
