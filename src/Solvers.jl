
function solve_displacements(setup::StaticSetup{T}, gsm::Matrix{T}) where T
    # setup.forces' index corresponds to the vertex id and contains a Vector2D
    force_vec = force_vector(setup)
    m = constrained_dof_array_mapping(setup)
    cfv = constrained_array(force_vec, m)
    cgsm = constrained_array(gsm, m)
    
    displacements = zeros(eltype(force_vec), length(force_vec))
    d_view = constrained_array(displacements, m)

    d_view .= cgsm \ cfv # this solves cgsm * displacements = cfv, or N/m * m = N, thus the displacements are N
    d_real = original_array(d_view)

    displacement_vecs = Vector2D{T}[Vector2D{T}(d_real[i], d_real[i+1]) for i in 1:2:length(d_real)]

    return displacement_vecs
end


"""
    solve_displacements(setup)

Solve the distances that each joint moves to keep the system stationary. The results of this function are necessary for most other properties that can be calculated. 
"""
solve_displacements(setup::StaticSetup) = solve_displacements(setup, global_stiffness_matrix(setup))

"""
    equilibrium_positions(setup, displacements)

Given a setup and its displacements, calculate the equilibrium positions of all of the joints. Returned indices match joint indices.  
"""
equilibrium_positions(s::StaticSetup, displacements::Vector{<:Vector2D}) = s.positions .+ displacements

"""
    solve_member_stresses(setup, displacements)
    
Using already calculated `displacements`, determine the stresses each member undergoes when at equilibrium. 
Compressive stresses are *negative and tensile stresses are *positive*, indexes match member ids.
""" 
function solve_member_stresses(setup::StaticSetup{T}, displacements::Vector{<:Vector2D}) where T
    fs = zeros(T, n_members(setup))
    for edge_id in member_ids(setup)
        v1, v2 = terminal_joints(setup, edge_id)
        pair1, _ = induced_forces(setup, edge_id, displacements)
        f1 = pair1.second
        fx1, fy1 = f1
        mag = norm(f1) # N

        x1, y1 = equilibriumxy(setup, displacements, v1)
        x2, y2 = equilibriumxy(setup, displacements, v2)
        
        δp = x2 - x1
        s = sign(fx1) * sign(x2 - x1)
        if δp ≈ 0
            δp += y2 - y1
            s = sign(fy1) * sign(y2 - y1)
        end
        fs[edge_id] = mag * s # N
    end
    return fs
end

"""
    solve_reaction_forces(setup, displacements)

Using already calculated `displacements`, find the forces that each constrained node needs to keep the system stationary. 
"""
function solve_reaction_forces(setup::StaticSetup{T}, displacements::Vector{<:Vector2D}) where T
    reaction_forces = zeros(Vector2D{T}, n_joints(setup))

    for mid in member_ids(setup)
        pair1, pair2 = induced_forces(setup, mid, displacements)
        v1, f1 = pair1
        v2, f2 = pair2
        reaction_forces[v1] += f1
        reaction_forces[v2] += f2
    end

    for (id, force) in enumerate(setup.forces)
        reaction_forces[id] -= force
    end
    return reaction_forces
end