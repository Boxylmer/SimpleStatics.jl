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
solve_displacements(setup::StaticSetup) = solve_displacements(setup, global_stiffness_matrix(setup))

equilibrium_positions(s::StaticSetup, displacements::Vector{<:Vector2D}) = s.positions .+ displacements

"Negative -> Compressive, Positive -> Tensile, index matches member index"
function solve_member_stresses(setup::StaticSetup{T}, displacements::Vector{<:Vector2D}) where T
    fs = zeros(T, n_members(setup))
    for edge_id in member_ids(setup)
        # local refers to the member, these are still in global coordinates
        k_local = member_stiffness_matrix(setup, edge_id)
        v1, v2 = terminal_joints(setup, edge_id)
        δx1, δy1 = displacements[v1]
        δx2, δy2 = displacements[v2]
        u_local = [δx1, δy1, δx2, δy2]
        fx1, fy1, fx2, fy2 = k_local * u_local
        x1, y1 = equilibriumxy(setup, displacements, v1)
        x2, y2 = equilibriumxy(setup, displacements, v2)
        
        δp = x2 - x1
        s = sign(fx1) * sign(x2 - x1)
        if δp ≈ 0
            δp += y2 - y1
            s = sign(fy1) * sign(y2 - y1)
        end
        mag = sqrt(fx1^2 + fy1^2)
        fs[edge_id] = mag * s
    end
    return fs
end
