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

function plot_region!(region::Polyhedron, plt::Plots.Plot = current(); kwargs...)
    xs, ys = Float64[], Float64[]
    for point in points(vrep(region))
        push!(xs, point[1])
        push!(ys, point[2])
    end
    plot!(Shape(xs, ys); kwargs...)
end
