function mccormick_envelope_constraints(model::Model, w::Variable, u::Variable, v::Variable, umin, umax, vmin, vmax)
    # convex approximation of w == u * v on u ∈ [umin, umax], v ∈ [vmin, vmax]
    vertices = [umin vmin umin * vmin; umax vmax umax * vmax; umax vmin umax * vmin; umin vmax umin * vmax]
    vr = SimpleVRepresentation(vertices)
    hr = SimpleHRepresentation(hrep(polyhedron(vr, CDDLibrary())))
    hrep_to_constraints(model, hr, [u; v; w])
end
