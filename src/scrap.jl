# @variables(model, begin
#     z[i = 1 : ncontacts, j = 1 : nregions, n = 1 : nsteps], Bin
#     r̂[k = 1 : 2, i = 1 : ncontacts, j = 1 : nregions, n = 1 : nsteps]
#     f̂[k = 1 : 2, i = 1 : ncontacts, j = 1 : nregions, n = 1 : nsteps]
# end)
# for j = 1 : nregions
#     region = regions[j]
#     for i = 1 : ncontacts, n = 1 : nsteps
#         polyhedron_constraints(model, region.position, r̂[:, i, j, n])
#         polyhedron_constraints(model, region.force, f̂[:, i, j, n])
#     end
#     lr, ur = axis_aligned_bounding_box(region.position)
#     lf, uf = axis_aligned_bounding_box(region.force)
#     @constraints(model, begin
#         [i = 1 : ncontacts, k = 1 : 2, n = 1 : nsteps], z[i, j, n] * lr[k] <= r̂[k, i, j, n]
#         [i = 1 : ncontacts, k = 1 : 2, n = 1 : nsteps], r̂[k, i, j, n] <= z[i, j, n] * ur[k]
#         [i = 1 : ncontacts, k = 1 : 2, n = 1 : nsteps], z[i, j, n] * lf[k] <= f̂[k, i, j, n]
#         [i = 1 : ncontacts, k = 1 : 2, n = 1 : nsteps], f̂[k, i, j, n] <= z[i, j, n] * uf[k]
#     end)
# end
# @constraints(model, begin
#     [i = 1 : ncontacts, k = 1 : 2, n = 1 : nsteps], sum(r̂[k, i, j, n] for j = 1 : nregions) == rc[k, i, n]
#     [i = 1 : ncontacts, k = 1 : 2, n = 1 : nsteps], sum(f̂[k, i, j, n] for j = 1 : nregions) == f[k, i, n]
#     [i = 1 : ncontacts, n = 1 : nsteps], sum(z[i, j, n] for j = 1 : nregions) == 1
# end)
