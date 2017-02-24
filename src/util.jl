perp(x) = [-x[2]; x[1]]

function axis_aligned_bounding_box(rep::VRepresentation)
    n = fulldim(rep)
    ps = points(rep)
    l = [minimum(p[i] for p in ps) for i = 1 : n]
    u = [maximum(p[i] for p in ps) for i = 1 : n]
    l, u
end

function axis_aligned_bounding_box(rep::HRepresentation)
    axis_aligned_bounding_box(vrep(polyhedron(rep, CDDLibrary())))
end

function hrep_to_constraints(model::Model, hr::HRepresentation, vars::Vector{Variable})
    for ineq in ineqs(hr)
        @constraint(model, ineq.a ⋅ vars <= ineq.β)
    end
    for eq in eqs(hr)
        @constraint(model, eq.a ⋅ vars == eq.β)
    end
end

# TODO: remove once Polyhedra.(*) is fixed
function cartesian_product(p1::SimpleHRepresentation, p2::SimpleHRepresentation)
    A = cat([1, 2], p1.A, p2.A)
    b = vcat(p1.b, p2.b)
    ls = linset(p1)
    for l in linset(p2)
        push!(ls, l + size(p1.A, 2))
    end
    SimpleHRepresentation(A, b, ls)
end

function mccormick_envelope_constraints(model::Model, w::Variable, u::Variable, v::Variable, umin, umax, vmin, vmax)
    # convex approximation of w == u * v on u ∈ [umin, umax], v ∈ [vmin, vmax]
    vertices = [umin vmin umin * vmin; umax vmax umax * vmax; umax vmin umax * vmin; umin vmax umin * vmax]
    vr = SimpleVRepresentation(vertices)
    hr = SimpleHRepresentation(hrep(polyhedron(vr, CDDLibrary())))
    hrep_to_constraints(model, hr, [u; v; w])
end

function hull_reformulation{HR<:HRepresentation}(model::Model, regions::Vector{HR}, x::AbstractVector{Variable})
    n = length(x)
    R = length(regions)
    @boundscheck begin
        for region in regions
            # ensure that dimension of region matches number of variables
            @assert fulldim(region) == n

            # ensure that there are no rays, so that setting the bounds of the inequalities to zero will
            # result in only the origin being inside the polyhedron
            vr = vrep(polyhedron(region))
            @assert nrays(vr) == 0
        end
    end

    Hrs = @variable(model, [r = 1 : R], Bin) # indicator variables, 4.2c
    @constraint(model, sum(Hrs) == 1) # 4.2b

    xrs = [@variable(model, [k = 1 : n]) for r = 1 : R] # auxiliary variables
    @constraint(model, x .== sum(xrs)) # 4.4a

    for (region, Hr, xr) in zip(regions, Hrs, xrs)
        for ineq in ineqs(region)
            @constraint(model, ineq.a ⋅ xr <= Hr * ineq.β) # 4.4b
        end
        for eq in eqs(region)
            @constraint(model, eq.a ⋅ xr == Hr * eq.β) # 4.4b
        end
    end
    Hrs
end

macro axis_variables(m, var, axes...)
    ranges = map(axis -> :(1 : length($axis)), axes)
    ret = quote
        vars = @variable $m $var[$(ranges...)]
        $(Expr(:call, :AxisArray, :vars, axes...))
    end
    esc(ret)
end

# TODO: use in test:
# function mccormick_envelope_constraints(model::Model, w::Variable, u::Variable, v::Variable, umin, umax)
#     # convex approximation of w == u * v for u ∈ [umin, umax], v ∈ [0, 1]
#     # 4.21
#     @constraints(model, begin
#         w >= umin * v
#         w >= umax * v + u - umax
#         w <= umin * v + u - umin
#         w <= umax * v
#         umin <= u
#         u <= umax
#     end)
# end

# function mccormick_envelope_constraints(model::Model, w::Variable, u::Variable, v::Variable)
#     # convex approximation of w == u * v for u ∈ [0, 1], v ∈ [0, 1]
#     # 4.20
#     for u ∈ [0, 1], v ∈ [0, 1]
#     @constraints(model, begin
#         w <= v
#         w <= u
#         w >= u + v - 1
#         0 <= u
#         u <= 1
#         0 <= v
#         v <= 1
#         0 <= w
#     end)
# end
