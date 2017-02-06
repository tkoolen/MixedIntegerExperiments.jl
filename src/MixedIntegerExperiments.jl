module MixedIntegerExperiments

using JuMP

include("util.jl")
include("region_descriptions.jl")

export
    RegionDescription,
    EnvironmentRegionDescription,
    contact_region,
    axis_aligned_free_box_region

end # module
