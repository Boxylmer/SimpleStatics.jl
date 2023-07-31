using SimpleStatics
function example_setup_1()
    # Matrix Analysis of Structures, chapter 3, example 3.7
    s = StaticSetup(Float64)
    j1 = add_joint!(s, Vector2D(0.0, 0.0))
    j2 = add_joint!(s, Vector2D(6.0, 0.0))
    j3 = add_joint!(s, Vector2D(0.0, 8.0))
    j4 = add_joint!(s, Vector2D(6.0, 8.0))
    m = StaticMaterial(0.0015, 70e6)
    m1 = add_member!(s, j1, j4, m)
    m2 = add_member!(s, j2, j4, m)
    m3 = add_member!(s, j3, j4, m)

    set_constraint!(s, j1, XRollerConstraint())
    set_constraint!(s, j2, AnchorConstraint())
    set_constraint!(s, j3, AnchorConstraint())
    set_force!(s, j1, Vector2D(-1000., 0.))
    return s
end

s1 = example_setup_1()
plot_setup(s1, "1 - Setup")
m = constrained_dof_array_mapping(s1)
gsm1 = global_stiffness_matrix(s1)
cgsm1 = constrained_array(gsm1, m) 
d1 = solve_displacements(s1) 
st1 = solve_member_stresses(s1, d1)
plot_setup(s1, "1 -Solution"; displacements=d1, stresses=st1)
