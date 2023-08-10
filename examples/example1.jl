using SimpleStatics
function example_setup_1()
    # Matrix Analysis of Structures, chapter 3, example 3.7
    s = StaticSetup(Float64)
    j1 = add_joint!(s, 0.0, 0.0)
    j2 = add_joint!(s, 6.0, 0.0)
    j3 = add_joint!(s, 0.0, 8.0)
    j4 = add_joint!(s, 6.0, 8.0)
    m1 = add_member!(s, j1, j4)
    m2 = add_member!(s, j2, j4)
    m3 = add_member!(s, j3, j4)

    set_constraint!(s, j1, XRollerConstraint())
    set_constraint!(s, j2, AnchorConstraint())
    set_constraint!(s, j3, AnchorConstraint())
    set_force!(s, j1, -1000., 0.)
    return s
end

s1 = example_setup_1()
plot_setup(s1, "1 - Setup")
gsm1 = global_stiffness_matrix(s1)
d1 = solve_displacements(s1) 
st1 = solve_member_stresses(s1, d1)
rf1 = solve_reaction_forces(s1, d1)

plot_setup(s1, "1 -Solution"; displacements=d1, stresses=st1, reactions=rf1)
