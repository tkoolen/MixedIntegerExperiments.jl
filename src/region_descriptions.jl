immutable RegionDescription{T}
    A::Matrix{T}
    b::Vector{T}
    l::Vector{T}
    u::Vector{T}
end

immutable EnvironmentRegionDescription{T}
    position::RegionDescription{T}
    force::RegionDescription{T}
end

# Forces
function friction_cone_constraints(n, μ, fnmax)
    n⟂ = perp(n)
    β1 = normalize(n + μ * n⟂)
    β2 = normalize(n - μ * n⟂)
    Af = [-perp(β1) perp(β2)]'
    bf = zeros(2)
    Af, bf
    fₗ = min.(β1 * fnmax, β2 * fnmax, zeros(2))
    fᵤ = max.(β1 * fnmax, β2 * fnmax, zeros(2))
    RegionDescription(Af, bf, fₗ, fᵤ)
end

function no_force_constraints()
    Af = [eye(2); -eye(2)]
    bf = zeros(4)
    fₗ = zeros(2)
    fᵤ = zeros(2)
    RegionDescription(Af, bf, fₗ, fᵤ)
end

# Positions
function line_segment_constraints(p1, p2)
    v = normalize(p2 - p1)
    n = perp(v)

    # vᵀ (r - p1) ≥ 0 ⇒ vᵀ r ≥ vᵀ p1
    # vᵀ (r - p1) ≤ 1 ⇒ -vᵀ r ≥ -(1  + vᵀ p1)
    # nᵀ (r - p1) ≥ 0 ⇒ nᵀ r ≥ nᵀ p1
    # nᵀ (r - p1) ≤ 0 ⇒ -nᵀ r ≥ -nᵀ p1

    Ar = [v'; -v'; n'; -n']
    br = [v ⋅ p1; -(1 + v ⋅ p1); n ⋅ p1; -n ⋅ p1]
    rₗ = min.(p1, p2)
    rᵤ = max.(p1, p2)
    RegionDescription(Ar, br, rₗ, rᵤ)
end

function axis_aligned_free_box(p1, p2, xaxis)
    zaxis = perp(xaxis)
    rₗ = min.(p1, p2)
    rᵤ = max.(p1, p2)
    Ar = [xaxis'; -xaxis'; zaxis'; -zaxis']
    br = [xaxis ⋅ rₗ; -xaxis ⋅ rᵤ; zaxis ⋅ rₗ; -zaxis ⋅ rᵤ]
    RegionDescription(Ar, br, rₗ, rᵤ)
end

# Position × Force environment regions
function contact_region(p1, p2, μ, fnmax)
    n = normalize(perp(p2 - p1))
    position = line_segment_constraints(p1, p2)
    force = friction_cone_constraints(n, μ, fnmax)
    EnvironmentRegionDescription(position, force)
end

function axis_aligned_free_box_region(p1, p2, xaxis)
    position = axis_aligned_free_box(p1, p2, xaxis)
    force = no_force_constraints()
    EnvironmentRegionDescription(position, force)
end
