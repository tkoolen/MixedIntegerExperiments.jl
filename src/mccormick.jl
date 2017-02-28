function mccormick_envelope(model::Model, w::Variable, u::Variable, v::Variable, urange::ClosedInterval, vrange::ClosedInterval)
    # convex approximation of w == u * v on u ∈ [umin, umax], v ∈ [vmin, vmax]
    umin, umax = minimum(urange), maximum(urange)
    vmin, vmax = minimum(vrange), maximum(vrange)
    vertices = [umin vmin umin * vmin; umax vmax umax * vmax; umax vmin umax * vmin; umin vmax umin * vmax]
    SimpleHRepresentation(hrep(polyhedron(SimpleVRepresentation(vertices), CDDLibrary())))
end

function piecewise_mccormick_envelope_constraints(model::Model, u::Variable, v::Variable, w::Variable,
    urange::Range, vrange::ClosedInterval)
    vars = [u; v; w]
    if length(urange) == 2
        envelope = mccormick_envelope(model, u, v, w, first(urange)..last(urange), vrange)
        hrep_to_constraints(model, hr, vars)
        nothing
    else
        N = length(urange) - 1
        envelopes = Vector{SimpleHRepresentation}(N)
        for i = 1 : N
            envelopes[i] = mccormick_envelope(model, u, v, w, urange[i]..urange[i + 1], vrange)
        end
        hull_reformulation(model, envelopes, vars)
        nothing
    end
end
