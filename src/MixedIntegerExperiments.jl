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
    EnvironmentRegionDescription,
    axis_aligned_bounding_box,
    contact_region,
    axis_aligned_free_box_region,
    plot_polyhedron!,
    hrep_to_constraints,
    mccormick_envelope_constraints,
    hull_reformulation,
    @axis_variables

end # module
