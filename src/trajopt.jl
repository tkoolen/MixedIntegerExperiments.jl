@with_kw immutable MIQPTrajOptParams
    nsteps::Int = 10
    Δt::Float64 = 0.1
    bilinearmethod::Symbol = :Logarithmic1D
    disc_level::Int = 9
    verbose::Bool = true
end

immutable ContactPointDescription{N}
    max_vel::Float64
    kinematic_region::SimpleVRepresentation{N,Float64}
end

immutable BoxRobotWithRotation2D
    θmax::Float64
    I::Float64
    m::Float64
    g::SVector{2, Float64}
    contact_point_descriptions::Dict{Symbol, ContactPointDescription{2}}
end

# max_angle(robot::BoxRobotWithRotation2D) = robot.θmax
# moment_of_inertia(robot::BoxRobotWithRotation2D) = robot.I
# mass(robot::BoxRobotWithRotation2D) = robot.m
# gravitational_acceleration(robot::BoxRobotWithRotation2D) = robot.g
# limb_descriptions(robot::BoxRobotWithRotation2D) = robot.contact_point_descriptions

immutable ContactPointState{N, T}
    pos::SVector{N, T}
    # TODO: contact info
end

immutable ContactPointInput{N, T}
    vel::SVector{N, T}
    force::SVector{N, T}
end

immutable BoxRobotWithRotation2DState{T}
    θ::T
    ω::T
    r::SVector{2, T}
    ṙ::SVector{2, T}
    contact_point_states::Dict{Symbol, ContactPointState{2, T}}
end

immutable MIQPTrajOptDiagnostics{A<:AbstractArray}
    solvetime::Float64
    total_torque_constraint_violation::A
    num_continuous_variables::Int
    num_binary_variables::Int
end

function rms_constraint_violation(diagnostics::MIQPTrajOptDiagnostics)
    n = length(diagnostics.total_torque_constraint_violation)
    sqrt(1 / n * sum(x^2 for x in diagnostics.total_torque_constraint_violation))
end

function miqp_trajopt{E<:EnvironmentRegion}(robot::BoxRobotWithRotation2D, environment::AbstractVector{E},
        initialstate::BoxRobotWithRotation2DState, params::MIQPTrajOptParams)
    nsteps = params.nsteps
    h = params.Δt
    bilinearmethod = params.bilinearmethod
    disc_level = params.disc_level

    coords = Axis{:coord}([:x, :z])
    steps = Axis{:step}(1 : nsteps)
    contacts = Axis{:contact}(collect(keys(robot.contact_point_descriptions)))
    regions = Axis{:region}(1 : length(environment))
    ncontacts = length(contacts)
    nregions = length(regions)

    r0 = initialstate.r
    ṙ0 = initialstate.ṙ
    θ0 = initialstate.θ
    ω0 = initialstate.ω

    θmax = robot.θmax
    I = robot.I
    m = robot.m
    g = robot.g

    model = if bilinearmethod == :minlp
        Model(solver = OsilSolver(solver = "couenne"))
    else
        Model(solver = GurobiSolver(Presolve = 0, OutputFlag = Int(params.verbose)))
    end

    rs = @axis_variable(model, r[coords, steps])
    θs = @axis_variable(model, θ[steps])
    ṙs = @axis_variable(model, ṙ[coords, steps])
    ωs = @axis_variable(model, ω[steps])
    rcs = @axis_variable(model, rc[coords, contacts, steps])
    ṙcs = @axis_variable(model, ṙc[coords, contacts, steps])
    fs = @axis_variable(model, f[coords, contacts, steps])
    Fs = @axis_variable(model, F[coords, steps])
    Ts = @axis_variable(model, T[steps])
    ss = @axis_variable(model, s[coords, contacts, steps])
    wzxs = @axis_variable(model, wzx[contacts, steps])
    wxzs = @axis_variable(model, wxz[contacts, steps])
    γs = @axis_variable(model, γ[contacts, regions, steps(1 : length(steps) - 1)])
    Γs = @axis_variable(model, Γ[contacts, regions])

    # Determine contact force bounds (needed for relaxation of bilinear constraints)
    fmin, fmax = force_bounds(environment)
    for i = 1 : ncontacts, n = 1 : nsteps
        f = fs[contacts(i), steps(n)]
        setlowerbound.(f, fmin)
        setupperbound.(f, fmax)
    end

    # region constraints (4.6)
    zs = AxisArray(Array{Variable, 3}(ncontacts, nregions, nsteps), contacts, regions, steps)
    product_regions = collect(region.position * region.force for region in environment)
    for i = 1 : ncontacts, n = 1 : nsteps
        rc = rcs[contacts(i), steps(n)]
        f = fs[contacts(i), steps(n)]
        zs[contacts(i), steps(n)] = hull_reformulation(model, product_regions, [rc; f])
    end

    # contact point velocity limits
    for i = 1 : length(contacts), n = 1 : length(steps)
        ṙc = ṙcs[contacts(i), steps(n)]

        zfree = JuMP.AffExpr(0)
        for j = 1 : length(regions)
            isfree(environment[j]) && (zfree += zs[contacts(i), steps(n), regions(j)]) # TODO: no need to do this in the inner loop
        end

        max_contact_point_vel = robot.contact_point_descriptions[contacts[i]].max_vel
        @constraints(model, begin
            -max_contact_point_vel * zfree .<= ṙc
            ṙc .<= max_contact_point_vel * zfree
        end)
    end

    # define s variables (contact point locations relative to CoM) (4.13)
    for i = 1 : ncontacts, n = 1 : nsteps
        s = ss[contacts(i), steps(n)]
        rc = rcs[contacts(i), steps(n)]
        r = rs[steps(n)]
        @constraint(model, s .== rc - r)
    end

    # kinematic constraints
    # setlowerbound.(rs[:z], 0.55) # TODO
    setlowerbound.(θs, -θmax)
    setupperbound.(θs, θmax)

    # contact point positions relative to CoM
    # NOTE: adding these increased solve time a lot
    for i = 1 : ncontacts, n = 1 : nsteps
        s = ss[contacts(i), steps(n)]
        polyhedron_constraints(model, robot.contact_point_descriptions[contacts[i]].kinematic_region, s)
    end

    # centroidal dynamics (4.18)
    for n = 1 : nsteps - 1
        @constraints(model, begin
            rs[steps(n + 1)] .== rs[steps(n)] + h * ṙs[steps(n + 1)]
            θs[steps(n + 1)] == θs[steps(n)] + h * ωs[steps(n + 1)]
            ṙs[steps(n + 1)] .== ṙs[steps(n)] + h / m * Fs[steps(n + 1)]
            ωs[steps(n + 1)] == ωs[steps(n)] + h / I * Ts[steps(n + 1)]
        end)
    end

    for n = 1 : nsteps
        @constraints(model, begin
            Fs[steps(n)] .== m * g + sum(fs[steps(n), contacts(i)] for i = 1 : length(contacts))
            Ts[steps(n)] == sum(wzxs[steps(n), contacts(i)] - wxzs[steps(n), contacts(i)] for i = 1 : length(contacts))
        end)
    end

    # contact point dynamics
    for i = 1 : ncontacts, n = 1 : nsteps - 1
        @constraints(model, begin
            rcs[contacts(i), steps(n + 1)] .== rcs[contacts(i), steps(n)] + h * ṙcs[contacts(i), steps(n + 1)]
        end)
    end

    # convex bilinear term approximations (4.12, 4.13)
    for i = 1 : ncontacts
        kinematic_region = robot.contact_point_descriptions[contacts[i]].kinematic_region
        sbounds = axis_aligned_bounding_box(kinematic_region) # needed for relaxation of bilinear terms
        smin = AxisArray(sbounds[1], coords)
        smax = AxisArray(sbounds[2], coords)

        for n = 1 : nsteps
            wxz = wxzs[contacts(i), steps(n)]
            wzx = wzxs[contacts(i), steps(n)]
            s = ss[contacts(i), steps(n)]
            f = fs[contacts(i), steps(n)]
            setlowerbound.(s, smin)
            setupperbound.(s, smax)
            if bilinearmethod == :HRepConvexHull
                srange = AxisArray(linspace.(smin, smax, disc_level), coords)
                fmin, fmax = AxisArray(getlowerbound.(f), coords), AxisArray(getupperbound.(f), coords)
                piecewise_mccormick_envelope_constraints(model, s[:x], f[:z], wxz, srange[:x], fmin[:z]..fmax[:z])
                piecewise_mccormick_envelope_constraints(model, s[:z], f[:x], wzx, srange[:z], fmin[:x]..fmax[:x])
            else
                @constraint(model, wxz == s[:x] * f[:z])
                @constraint(model, wzx == s[:z] * f[:x])
            end
        end
    end

    # initial conditions
    @constraints(model, begin
        rs[steps(1)] .== r0
        θs[steps(1)] == θ0
        ṙs[steps(1)] .== ṙ0
        ωs[steps(1)] == ω0
    end)

    for i = 1 : ncontacts
        rc0 = initialstate.contact_point_states[contacts[i]].pos
        @constraint(model, rcs[contacts(i), steps(1)] .== rc0)
        # TODO: initial region assignments (maybe).
    end

    # final conditions
    @constraints(model, begin
        # rs[steps(length(steps))] .== [0.81; 0.9]
        θs[steps(length(steps))] == 0
        ṙs[steps(length(steps))] .== 0
        ωs[steps(length(steps))] == 0
    end)

    # section 4.4
    # objective function + slack parameterization
    @constraint(model, Γs .== sum(γs[steps(n)] for n = 1 : length(steps) - 1));
    for n = 1 : length(steps) - 1
        @constraints(model, begin
            -γs[steps(n)] .<= zs[steps(n + 1)] - zs[steps(n)]
            zs[steps(n + 1)] - zs[steps(n)] .<= γs[steps(n)]
        end)
    end
    setlowerbound.(γs, 0.)
    setupperbound.(γs, 1.)

    δs = rs .- r0
    @objective(model, Min, 1e3 * sum(Γ) + 1e-3 * vecdot(Fs, Fs) + 1e-5 * vecdot(fs, fs) + 1e-2 * vecdot(δs, δs) + vecdot(θs, θs))

    # relax bilinear constraints
    if bilinearmethod ∉ [:HRepConvexHull; :minlp]
        relaxbilinear!(model, method = bilinearmethod, disc_level = disc_level)
    end

    # solve
    status, solvetime = @timed solve(model)
    @assert status == :Optimal

    # compile results
    # states
    states = AxisArray(Vector{BoxRobotWithRotation2DState{Float64}}(length(steps)), steps)
    for n = 1 : nsteps
        θ = getvalue(θs[steps(n)])
        ω = getvalue(ωs[steps(n)])
        r = SVector{2}(getvalue.(rs[steps(n)]))
        ṙ = SVector{2}(getvalue.(ṙs[steps(n)]))

        contact_point_states = Dict{Symbol, ContactPointState{2, Float64}}()
        for i = 1 : ncontacts
            rc = SVector{2}(getvalue.(rcs[contacts(i), steps(n)]))
            contact_point_states[contacts[i]] = ContactPointState(rc)
        end

        states[steps(n)] = BoxRobotWithRotation2DState(θ, ω, r, ṙ, contact_point_states)
    end

    # inputs
    createinput = (contact, step) -> begin
        ṙc = SVector{2}(getvalue(ṙcs[contact, step]))
        f = SVector{2}(getvalue(fs[contact, step]))
        input = ContactPointInput(ṙc, f)
    end
    # inputs = Dict(contacts[i] => AxisArray(map(n -> createinput(contacts(i), steps(n)), 1 : nsteps), steps) for i = 1 : ncontacts)
    inputs = AxisArray(map(n -> Dict(contacts[i] => createinput(contacts(i), steps(n)) for i = 1 : ncontacts), 1 : nsteps), steps)

    # diagnostics
    zx_constraint_violations = getvalue(AxisArray(wzxs .- (ss[:z] .* fs[:x]), wzxs.axes))
    xz_constraint_violations = getvalue(AxisArray(wxzs .- (ss[:x] .* fs[:z]), wxzs.axes))
    torque_constraint_violations = AxisArray(vec(sum((zx_constraint_violations - xz_constraint_violations), 1)), steps) # TODO: use Axes

    num_continuous_variables = sum(model.colCat .== :Cont)
    num_binary_variables = sum(model.colCat .== :Bin)
    diagnostics = MIQPTrajOptDiagnostics(solvetime, torque_constraint_violations, num_continuous_variables, num_binary_variables)

    states, inputs, diagnostics
end
