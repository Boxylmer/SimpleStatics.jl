using Revise
using SimpleStatics
using Plots

function build_truss(width_m, height_m, n, top_load = 0; 
    thick_material=Materials.SquareTubing(Materials.MildSteel, 1.5 * 0.0381, 1.897 / 1000),   # 1.5" steel tubing with 14 gauge thickness
    medium_material=Materials.SquareTubing(Materials.MildSteel, 1.0 * 0.0381, 1.518 / 1000),  # 1.0" steel tubing with 16 gauge thickness
    thin_material=Materials.SquareTubing(Materials.MildSteel, 0.75 * 0.0381, 1.518 / 1000)    # .75" steel tubing with 16 gauge thickness
    )

    s = StaticSetup()
    
    segment_len = width_m / (2*n)
    
    top_joint_indices = []
    for m in range(0, width_m, 2n + 1)
        j = add_joint!(s, m, height_m)
        push!(top_joint_indices, j)
    end
    
    bottom_joint_indices = []
    for m in range(segment_len, width_m - segment_len, n)
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

    cross_members = []
    for i in 1:n
        for j in -1:1
            m = add_member!(s, bottom_joint_indices[i], top_joint_indices[2*i + j], thin_material)
            push!(cross_members, m)
        end
    end
    set_constraint!(s, top_joint_indices[1], AnchorConstraint())
    set_constraint!(s, top_joint_indices[end], XRollerConstraint())
    
    SimpleStatics.add_member_weights!(s)
    # add_force!(s, top_joint_indices[n+1], 0, -top_load) # middle of top bar
    for i in top_joint_indices
        add_force!(s, i, 0, -top_load/length(top_joint_indices)) 
    end
    return s
end


width = (23 * 12 + 10) * 0.0254 # 23', 10" long -> Meters
height = 18 * 0.0254 # 18" tall -> Meters
s1 = build_truss(width, height, 4)
load = 500 * 0.453592 * 9.81 # N

nvals = 1:10
trusses = [build_truss(width, height, nval, load) for nval in nvals]
displacements = solve_displacements.(trusses)
forces = solve_member_forces.(trusses, displacements)
reactions = solve_reaction_forces.(trusses, displacements)
member_stresses = solve_member_stresses.(trusses, forces)

maximum(member_stresses[4])

max_stresses = maximum.(solve_stress_utilization.(trusses, member_stresses))
plot(nvals, max_stresses)
plot_setup(trusses[4]; dsize=3200, padding=0.4, displacements=displacements[4], member_forces=forces[4], reactions=reactions[4])
plot_setup(trusses[5]; dsize=3200, padding=0.4, displacements=displacements[5], member_forces=forces[5], reactions=reactions[5])
plot_setup(trusses[10]; dsize=3200, padding=0.4, displacements=displacements[10], member_forces=forces[10], reactions=reactions[10])

@show displacements[10][10].y * 39.3701
@show displacements[6][6].y * 39.3701
@show displacements[5][5].y * 39.3701

for (i, stress) in enumerate(rs1)
    mpa = stress / 1000000
    println("Stress at member $i: $mpa mpa")
end

println(mass(s1) * 2.20462, " lb")

print(maximum(solve_stress_utilization(s1, rs1)))

truss_masses = [mass(build_truss(width, height, i)) * 2.20462 for i in 1:10]
plot(1:10, truss_masses)