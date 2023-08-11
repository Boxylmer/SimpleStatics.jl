using SimpleStatics

function build_truss(width_m, height_m, n, load_lbf=0; 
    thick_material=Materials.SquareTubing(Materials.MildSteel, 1.5 * 0.0381, 1.897 / 1000),   # 1.5" steel tubing with 14 gauge thickness
    medium_material=Materials.SquareTubing(Materials.MildSteel, 1.0 * 0.0381, 1.518 / 1000),  # 1.0" steel tubing with 16 gauge thickness
    # thin_material=Materials.SquareTubing(Materials.MildSteel, 1.0 * 0.0381, 1.518 / 1000)
    )

    s = StaticSetup()
    
    m = width_m / (2*n)
    
    top_joint_indices = []
    for m in range(0, width_m, 2n + 1)
        j = add_joint!(s, m, height_m)
        push!(top_joint_indices, j)
    end
    
    bottom_joint_indices = []
    for m in range(m, width_m - m, n)
        j = add_joint!(s, m, 0)
        push!(bottom_joint_indices, j)
    end
    
    top_bar_members = []
    for i in top_joint_indices[1:end-1]
        m = add_member!(s, i, i+1)
        push!(top_bar_members, m, thick_material)
    end

    bottom_bar_members = []
    for i in bottom_joint_indices[1:end-1]
        m = add_member!(s, i, i+1)
        push!(bottom_bar_members, m, medium_material)
    end
    # @show bottom_bar_members

    cross_members = []
    for i in 1:n
        for j in -1:1
            m = add_member!(s, bottom_joint_indices[i], top_joint_indices[2*i + j], medium_material)
            push!(cross_members, m)
        end
    end
    # set_constraint!(s, j1, XRollerConstraint())
    set_constraint!(s, top_joint_indices[1], AnchorConstraint())
    set_constraint!(s, top_joint_indices[end], XRollerConstraint())
    # set_force!(s, j1, -1000., 0.)
    return s
end


n = 4
s1 = build_truss(10, 2, n)

truss_midpoint_joint = n + 1

SimpleStatics.add_member_weights!(s1)
# set_force!(s1, truss_midpoint_joint, 0, -1000)

plot_setup(s1, "1 - Setup", dsize=1600)
gsm1 = global_stiffness_matrix(s1)
d1 = solve_displacements(s1) 
st1 = solve_member_stresses(s1, d1)
rf1 = solve_reaction_forces(s1, d1)

plot_setup(s1, "1 -Solution"; dsize=1600, displacements=d1, stresses=st1, reactions=rf1)
