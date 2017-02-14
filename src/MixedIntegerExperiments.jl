module MixedIntegerExperiments

using JuMP
using Polyhedra, CDDLib

include("util.jl")
include("region_descriptions.jl")

export
    RegionDescription,
    EnvironmentRegionDescription,
    axis_aligned_bounding_box,
    contact_region,
    axis_aligned_free_box_region

end # module
