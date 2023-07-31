using SimpleStatics
function example_setup_4()
    # Matrix Analysis of Structures, chapter 4, truss in example 4.2
    s = StaticSetup(Float64)
    j1 = add_joint!(s, Vector2D(0.0, 0.0))
    j2 = add_joint!(s, Vector2D(16.0 * 12, 0.0 * 12))
    j3 = add_joint!(s, Vector2D(16.0 * 12, 12.0 * 12))
    j4 = add_joint!(s, Vector2D(32.0 * 12, 0.0 * 12))
    j5 = add_joint!(s, Vector2D(32.0 * 12, 12.0 * 12))

    m = StaticMaterial(10., 30000)
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
    set_force!(s, j2, Vector2D(0., -100.))
    set_force!(s, j5, Vector2D(50., 0.))
end