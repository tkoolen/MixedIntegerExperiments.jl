module MixedIntegerExperiments

using JuMP
using Polyhedra, CDDLib
using Plots
using DrakeVisualizer
using ColorTypes
using FixedSizeArrays
using AxisArrays

include("util.jl")
include("region_descriptions.jl")
include("disjoint_union.jl")
include("mccormick.jl")
include("plotting.jl")

export
    RegionDescription,
    EnvironmentRegion,
    axis_aligned_bounding_box,
    contact_region,
    isfree,
    axis_aligned_free_box_region,
    plot_polyhedron!,
    plot_environment,
    plot_allowable_forces,
    plot_kinematic_regions,
    polyhedron_constraints,
    mccormick_envelope,
    piecewise_mccormick_envelope_constraints,
    hull_reformulation,
    @axis_variables

end # module
