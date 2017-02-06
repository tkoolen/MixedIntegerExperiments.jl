using MixedIntegerExperiments
using Base.Test

import MixedIntegerExperiments: perp

for i = 1 : 10
    x = rand(2)
    x⟂ = perp(x)
    @test isapprox(x⟂ ⋅ x, 0., atol = 1e-14)
    @test isapprox(norm(x), norm(x⟂), atol = 1e-14)
end

for i = 1 :  10
    p1 = randn(2)
    p2 = randn(2)
    μ = rand()
    fmax = 20.
    region = contact_region(p1, p2, μ, fmax)
    n = normalize(perp(p2 - p1))
    @test all(region.force.A * fmax * (1 - eps()) * n .>= region.force.b)

    for j = 1 : 100
        f = randn(2)
        infrictioncone = norm(f) <= sqrt(1 + μ^2) * dot(f, n)
        @test all(region.force.A * f .>= region.force.b) == infrictioncone
    end
end

for i = 1 : 10
    p1 = rand(2)
    p2 = rand(2)
    xaxis = [1.; 0]
    region = axis_aligned_free_box_region(p1, p2, xaxis)

    for j = 1 : 100
        λ1 = rand()
        λ = [λ1; 1 - λ1]
        p = [p1 p2] * λ
        @test all(region.position.A * p .>= region.position.b)
    end
    for j = 1 : 100
        p = randn(2)
        λ1 = randn()
        p = λ1 * (p1 - p2) + p2
        @test (zero(λ1) <= λ1 <= one(λ1)) == all(region.position.A * p .>= region.position.b)
    end
end
