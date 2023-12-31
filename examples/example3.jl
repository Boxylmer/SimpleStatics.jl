using SimpleStatics
function example_setup_3()
    s = StaticSetup(Float64)
    j1 = add_joint!(s, 0.0, 0.0)
    j2 = add_joint!(s, 16.0 * 12, 0.0 * 12)
    j3 = add_joint!(s, 16.0 * 12, 12.0 * 12)
    j4 = add_joint!(s, 32.0 * 12, 0.0 * 12)
    j5 = add_joint!(s, 32.0 * 12, 12.0 * 12)

    m = StaticMaterial(10., 300)
    m1 = add_member!(s, j1, j3, m)
    m2 = add_member!(s, j1, j2, m)
    m3 = add_member!(s, j2, j3, m)
    m4 = add_member!(s, j3, j5, m)
    m5 = add_member!(s, j3, j4, m)
    m6 = add_member!(s, j2, j5, m)
    m7 = add_member!(s, j2, j4, m)
    m8 = add_member!(s, j4, j5, m)

    set_constraint!(s, j1, AnchorConstraint())
    set_constraint!(s, j4, AnchorConstraint())
    set_force!(s, j2, 0., 50.)
    set_force!(s, j5, -50., 0.)
    return s
end

s3 = example_setup_3()

plot_setup(s3, "3 - Setup"; dsize=1000)
gsm3 = global_stiffness_matrix(s3)
d3 = solve_displacements(s3) 
f3 = solve_member_forces(s3, d3)
rf3 = solve_reaction_forces(s3, d3)
plot_setup(s3, "3 - Solution"; displacements=d3, stresses=f3, reactions=rf3, padding=0.7)
