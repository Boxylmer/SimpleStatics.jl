using SimpleStatics
function example_setup_2()
    s = StaticsSetup(Float64)
    j1 = add_joint!(s, Vector2D(0.0, 0.0))
    j2 = add_joint!(s, Vector2D(1.0, 0.0))
    j3 = add_joint!(s, Vector2D(1.0, 1.0))
    j4 = add_joint!(s, Vector2D(2.0, 0.0))

    m1 = add_member!(s, j1, j2)
    m2 = add_member!(s, j1, j3)
    m3 = add_member!(s, j2, j3)
    m4 = add_member!(s, j2, j4)
    m5 = add_member!(s, j3, j4)

    set_constraint!(s, j1, AnchorConstraint())
    set_constraint!(s, j4, XRollerConstraint())
    set_force!(s, j3, Vector2D(5000., -3000.))
    return s
end

s2 = example_setup_2()
plot_setup(s2, "2 - Setup")
global_stiffness_matrix(s2) 
d2 = solve_displacements(s2)
st2 = solve_member_stresses(s2, d2)
plot_setup(s2, "2 - Solution", stresses=st2, displacements=d2)

