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

function polyhedron_constraints(model::Model, rep::HRepresentation, vars::AbstractVector{<:VariableRef})
    for ineq in ineqs(rep)
        @constraint(model, ineq.a ⋅ vars <= ineq.β)
    end
    for eq in eqs(rep)
        @constraint(model, eq.a ⋅ vars == eq.β)
    end
end

function polyhedron_constraints(model::Model, rep::VRepresentation, vars::AbstractVector{<:VariableRef})
    hr = SimpleHRepresentation(hrep(polyhedron(rep, CDDLibrary())))
    polyhedron_constraints(model, hr, vars)
end

JuMP.value(A::AxisArray) = AxisArray(JuMP.value(A.data), A.axes)

# TODO: use in test:
# function mccormick_envelope_constraints(model::Model, w::<:VariableRef, u::<:VariableRef, v::<:VariableRef, umin, umax)
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

# function mccormick_envelope_constraints(model::Model, w::<:VariableRef, u::<:VariableRef, v::<:VariableRef)
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
