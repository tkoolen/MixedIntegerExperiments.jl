immutable EnvironmentRegion{N, T}
    position::SimpleHRepresentation{N, T}
    force::SimpleHRepresentation{N, T}
end
dimension{N, T}(::Type{EnvironmentRegion{N, T}}) = N

function isfree(region::EnvironmentRegion)
    vr = vrep(polyhedron(region.force))
    !hasrays(vr) && all(all(p .== 0) for p in points(vr))
end

function lphrep_with_bounds(vr::VRepresentation)
    hr = hrep(polyhedron(vr, CDDLibrary()))
    lphr = LPHRepresentation(hr)
    l, u = axis_aligned_bounding_box(vr)
    LPHRepresentation(lphr.A, l, u, lphr.lb, lphr.ub)
end

# Forces
function friction_cone_constraints(n, μ, fnmax)
    n⟂ = perp(n)
    β1 = normalize(n + μ * n⟂) * fnmax
    β2 = normalize(n - μ * n⟂) * fnmax
    vr = SimpleVRepresentation([zeros(1, length(n⟂)); β1'; β2'])
    SimpleHRepresentation(hrep(polyhedron(vr, CDDLibrary())))
end

function no_force_constraints(n)
    SimpleHRepresentation(eye(2), zeros(2), IntSet([1, 2]))
end

# Positions
function line_segment_constraints(p1, p2)
    vr = SimpleVRepresentation([p1'; p2'])
    SimpleHRepresentation(hrep(polyhedron(vr, CDDLibrary())))
end

function axis_aligned_free_box(p1, p2)
    l = min.(p1, p2)
    u = max.(p1, p2)
    n = length(l)
    SimpleHRepresentation([eye(n); -eye(n)], [u; -l])
end

# Position × Force environment regions
function contact_region(p1, p2, μ, fnmax)
    n = normalize(perp(p2 - p1))
    position = line_segment_constraints(p1, p2)
    force = friction_cone_constraints(n, μ, fnmax)
    EnvironmentRegion(position, force)
end

function axis_aligned_free_box_region(p1, p2)
    position = axis_aligned_free_box(p1, p2)
    force = no_force_constraints(length(p1))
    EnvironmentRegion(position, force)
end

function force_bounds{T<:EnvironmentRegion}(environment::AbstractVector{T})
    # TODO: could do this on a contact-by-contact basis if certain contact-region assignments are not allowed
    n = dimension(T)
    fmin = zeros(n)
    fmax = zeros(n)
    for region in environment
        fl, fu = axis_aligned_bounding_box(region.force)
        fmin .= min.(fmin, fl)
        fmax .= max.(fmax, fu)
    end
    fmin, fmax
end
