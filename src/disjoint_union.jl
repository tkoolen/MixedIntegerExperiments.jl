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
