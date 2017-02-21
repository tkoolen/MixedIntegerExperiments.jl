function plot_polyhedron!(region::Polyhedron, plt::Plots.Plot = current(); kwargs...)
    xs, ys = Float64[], Float64[]
    for point in points(vrep(region))
        push!(xs, point[1])
        push!(ys, point[2])
    end
    plot!(Shape(xs, ys); kwargs...)
end

function plot_piecewise_mccormick(N, umin, umax, vmin, vmax)
    DrakeVisualizer.any_open_windows() || DrakeVisualizer.new_window()
    vis = Visualizer()

    wmin = minimum(u * v for u in (umin, umax), v in (vmin, vmax))
    wmax = maximum(u * v for u in (umin, umax), v in (vmin, vmax))
    f = uvw -> uvw[1] * uvw[2] - uvw[3]
    mesh = contour_mesh(f, Vec(umin, vmin, wmin), Vec(umax, vmax, wmax))
    green = RGBA(0., 1, 0, 1.0)
    red = RGBA(1., 0, 0, 1.0)
    setgeometry!(vis[:exact], GeometryData(mesh, green))

    for i = 1 : N
        m = Model();
        @variables(m, begin
            u
            v
            w
        end)
        umin_piece = umin + (i - 1) * (umax - umin) / N
        umax_piece = umin + i * (umax - umin) / N
        mccormick_envelope_constraints(m, w, u, v, umin_piece, umax_piece, vmin, vmax)
        poly = polyhedron(m, CDDLibrary(:exact))
        setgeometry!(vis[:mccormick][Symbol("piece$i")], GeometryData(poly, red))
    end
    vis
end
