function plot_polyhedron!(region::Polyhedron, plt = current(); kwargs...)
    xs, ys = Float64[], Float64[]
    cross2 = (p1, p2) -> p1[2] * p2[1] - p1[1] * p2[2]
    points_clockwise = sort(collect(points(vrep(region))), lt = (p1, p2) -> cross2(p1, p2) <= 0)
    for point in points_clockwise
        push!(xs, point[1])
        push!(ys, point[2])
    end
    plot!(Shape(xs, ys); kwargs...)
end

function plot_piecewise_mccormick(N, umin, umax, vmin, vmax)
    DrakeVisualizer.any_open_windows() || (DrakeVisualizer.new_window(); sleep(1))
    vis = Visualizer()

    wmin = minimum(u * v for u in (umin, umax), v in (vmin, vmax))
    wmax = maximum(u * v for u in (umin, umax), v in (vmin, vmax))
    f = uvw -> uvw[1] * uvw[2] - uvw[3]
    mesh = contour_mesh(f, Vec(umin, vmin, wmin), Vec(umax, vmax, wmax))
    green = RGBA(0., 1, 0, 1.0)
    red = RGBA(1., 0, 0, 0.5)
    setgeometry!(vis[:exact], GeometryData(mesh, green))

    for i = 1 : N
        m = Model()
        @variables(m, begin
            u
            v
            w
        end)
        umin_piece = umin + (i - 1) * (umax - umin) / N
        umax_piece = umin + i * (umax - umin) / N
        envelope = mccormick_envelope(m, u, v, w, umin_piece..umax_piece, vmin..vmax)
        polyhedron_constraints(m, envelope, [u; v; w])
        poly = polyhedron(m, CDDLibrary(:exact))
        setgeometry!(vis[:mccormick][Symbol("piece$i")], GeometryData(poly, red))
    end
    vis
end

function plot_environment{T<:EnvironmentRegion}(environment::AbstractVector{T})
    plt = plot()
    for region in environment
        kwargs = if isfree(region)
            ((:lab, "free"), (:color, "blue"), (:opacity, 0.1))
        else
            ((:lab, "contact"), (:fillcolor, "black"))
        end
        plot_polyhedron!(polyhedron(region.position); kwargs...)
    end
    title!(plt, "Environment")
    plt
end

function plot_allowable_forces(region::EnvironmentRegion, id)
    plt = plot()
    plot_polyhedron!(polyhedron(region.force); opacity = 0.5, lab = "")
    title!("Allowable contact forces, region $id")
    plt
end

function plot_kinematic_regions{T<:VRepresentation}(contact_kinematic_regions::Dict{Symbol, T})
    plt = plot()
    for (name, kinematic_region) in contact_kinematic_regions
        plot_polyhedron!(polyhedron(kinematic_region); lab = string(name), opacity = 0.5)
    end
    title!("Kinematic reachability relative to CoM")
    plt
end
