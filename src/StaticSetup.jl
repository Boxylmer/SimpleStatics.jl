struct StaticSetup{T}
    positions::Vector{<:Vector2D{T}} # Meters
    forces::Vector{<:Vector2D{T}} # newtons
    constraints::Vector{<:StaticConstraint}
    materials::Vector{<:StaticMaterial}  # 
    vertices_to_edge::Dict{UnorderedPair, Int32}
    edge_to_vertices::Vector{<:UnorderedPair}  
end 

"""
    StaticSetup(T::Type=Float64)

Create a blank StaticSetup where data is stored in type T. 
Measurement types are supported and tested for. 
Unitful types should be supported, but tests haven't been written for them yet. (todo)

"""
function StaticSetup(T::Type=Float64)
    positions = Vector{Vector2D{T}}() # vertex id -> position of that joint
    forces = Vector{Vector2D{T}}() # vertex id -> force vector acting on that joint
    constraints = Vector{StaticConstraint}() # vertex id -> constraint for that joint
    materials = Vector{StaticMaterial{T}}() # edge id -> material for that member
    vertices_to_edge = Dict{UnorderedPair, Int32}() # use two vertices ids to look up the edge id connecting them
    edge_to_vertices = Vector{UnorderedPair}() # use an edge id to get the vertices it joins
    return StaticSetup(positions, forces, constraints, materials, vertices_to_edge, edge_to_vertices)
end

Base.eltype(::StaticSetup{T}) where T = T 

"Get the total number of joints."
n_joints(s::StaticSetup) = length(s.positions)

"Get the total number of members."
n_members(s::StaticSetup) = length(s.materials)

"Get an iterator which traverses through all joint ids."
joint_ids(s::StaticSetup) = 1:n_joints(s)

"Get an iterator which traverses through all member ids."
member_ids(s::StaticSetup) = 1:n_members(s)

"Naive number of dergees of freedom in the system (i.e., before constraints)."
n_dofs(s::StaticSetup) = 2 * n_joints(s)

"Get the joints IDs that a member connects, found by its member ID."
terminal_joints(s::StaticSetup, member_id::Integer) = s.edge_to_vertices[member_id]


initialxy(s::StaticSetup, vid::Integer) = s.positions[vid].x, s.positions[vid].y
equilibriumxy(s::StaticSetup, displacements::Vector{<:Vector2D}, vid::Integer) = s.positions[vid].x + displacements[vid].x, s.positions[vid].y + displacements[vid].y

"Add a joint to the setup and return the index referring to that joint. The position should be set in meters."
function add_joint!(setup::StaticSetup{T}, position::Vector2D{T}, constraint::StaticConstraint=NoConstraint()) where T
    node_index = n_joints(setup) + 1
    push!(setup.positions, position)
    push!(setup.constraints, constraint)
    push!(setup.forces, Vector2D(T(0), T(0)))
    return node_index
end

add_joint!(setup::StaticSetup{T}, x::Number, y::Number, args...) where T = add_joint!(setup, Vector2D(convert(T, x), convert(T, y)), args...)

"Add a member to the setup and return the index referring to that member."
function add_member!(setup::StaticSetup, j1::Int, j2::Int, material::StaticMaterial{T}=Materials.PerfectMaterial()) where T
    if j1 == j2
        throw(ArgumentError("A member cannot connect a joint to itself."))
    end
    
    edge_key = UnorderedPair(j1, j2)
    if haskey(setup.vertices_to_edge, edge_key)
        throw(ArgumentError("A member between these nodes already exists."))
    end
    
    push!(setup.materials, material)
    n = length(setup.materials)
    setup.vertices_to_edge[edge_key] = n
    push!(setup.edge_to_vertices, edge_key)
    return n
end

"Set the force at a joint, in Newtons."
function set_force!(setup::StaticSetup, joint_index::Int, force::Vector2D)
    if typeof(setup.constraints[joint_index]) <: AnchorConstraint
        @warn "Force set on fully constrained joint! This won't cause anything to happen. Did you mean to place this force somewhere else?"
    end
    setup.forces[joint_index] = force
end

set_force!(setup::StaticSetup{T}, ji::Integer, fx::Number, fy::Number) where T = set_force!(setup, ji, Vector2D(convert(T, fx), convert(T, fy)))

"Add a force to the joint, in Newtons."
function add_force!(setup::StaticSetup, ji::Integer, force::Vector2D)
    setup.forces[ji] = setup.forces[ji] + force
end

add_force!(setup::StaticSetup{T}, ji::Integer, fx::Number, fy::Number) where T = add_force!(setup, ji, Vector2D(convert(T, fx), convert(T, fy)))

function set_constraint!(setup::StaticSetup, joint_index::Int, constraint::StaticConstraint)
    setup.constraints[joint_index] = constraint
end

function member_length(setup::StaticSetup, member_idx::Int)
    edge_key = setup.edge_to_vertices[member_idx]
    node_index1, node_index2 = edge_key.a, edge_key.b
            
    p1 = setup.positions[node_index1]
    p2 = setup.positions[node_index2]
    dx = p2.x - p1.x
    dy = p2.y - p1.y
            
    return sqrt(dx^2 + dy^2)
end

"Get the angle, in radians, the member makes with the global coordinate system's positive X axis."
function member_angle(setup::StaticSetup, edge_id::Int)
    edge_key = setup.edge_to_vertices[edge_id]
    node_index1, node_index2 = edge_key

    p1 = setup.positions[node_index1]
    p2 = setup.positions[node_index2]
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    return atan(dy, dx)
end

"Get the global stiffness matrix for a StaticSetup in Newtons / Meter."
function member_stiffness_matrix(setup::StaticSetup, edge_id)
    l = member_length(setup, edge_id) # meters
    edge_key = setup.edge_to_vertices[edge_id]
    node_index1, node_index2 = edge_key
    c = (setup.positions[node_index2].x - setup.positions[node_index1].x) / l
    s = (setup.positions[node_index2].y - setup.positions[node_index1].y) / l
    m = setup.materials[edge_id]
    A = m.area # m2
    E = m.modulus # N/m2
    scalar = A * E / l # m2 * N/m2 / m -> units: N / m
    return scalar * [
        c^2 c*s -c^2 -c*s;
        c*s s^2 -c*s -s^2;
       -c^2 -c*s c^2  c*s;
       -c*s -s^2 c*s  s^2;
    ]
end

"Get the indices of the x and y rows, respectively, in a square DOF-like matrix (e.g., global stiffness matrix) for a vertex (joint) index."
function dof_indices(node_index::Integer)
    row_start = 2 * node_index - 1
    row_end = 2 * node_index
    return row_start:row_end
end

"Get a vector of the unconstrained forces in the setup."
function force_vector(setup::StaticSetup)
    force_vec = zeros(eltype(setup), n_dofs(setup))
    for i in eachindex(setup.forces)
        force_vec[dof_indices(i)] .= setup.forces[i].x, setup.forces[i].y
    end
    return force_vec
end

"Get the stiffness matrix of the entire `setup` with respect to the global coordinate system."
function global_stiffness_matrix(setup::StaticSetup)
    n = n_dofs(setup)
    k_global = zeros(n, n)

    for edge_id in member_ids(setup)
        K_member = member_stiffness_matrix(setup, edge_id)

        # Get the DOF indices for the nodes of the current member
        edge_key = setup.edge_to_vertices[edge_id]
        node_index1, node_index2 = edge_key
        node_index1_indices = dof_indices(node_index1)
        node_index2_indices = dof_indices(node_index2)

        node_1_x, node_1_y = node_index1_indices
        node_2_x, node_2_y = node_index2_indices

        # diagonal terms 
        k_global[node_1_x, node_1_x] += K_member[1, 1]
        k_global[node_1_y, node_1_y] += K_member[2, 2]
        k_global[node_2_x, node_2_x] += K_member[3, 3]
        k_global[node_2_y, node_2_y] += K_member[4, 4]
                
        # Off-diagonal terms
        k_global[node_1_x, node_1_y] += K_member[1, 2]
        k_global[node_1_x, node_2_x] += K_member[1, 3]
        k_global[node_1_x, node_2_y] += K_member[1, 4]
        k_global[node_1_y, node_2_x] += K_member[2, 3]
        k_global[node_1_y, node_2_y] += K_member[2, 4]
        k_global[node_2_x, node_2_y] += K_member[3, 4]

        # The off-diagonal terms are symmetric, so we update both (i, j) and (j, i) entries
        k_global[node_1_y, node_1_x] += K_member[2, 1]
        k_global[node_2_x, node_1_x] += K_member[3, 1]
        k_global[node_2_y, node_1_x] += K_member[4, 1]
        k_global[node_2_x, node_1_y] += K_member[3, 2]
        k_global[node_2_y, node_1_y] += K_member[4, 2]
        k_global[node_2_y, node_2_x] += K_member[4, 3]
       
    end

    return k_global
end

"Find a mapping m[i] -> j where i is the index of a constrained dof array and j is the corresponding index of the unconstrained array."
function constrained_dof_array_mapping(setup)
    mapping = Vector{Int64}() 

    for node_index in joint_ids(setup)
        constraint = setup.constraints[node_index]
        xi, yi = dof_indices(node_index)
        
        if constraint isa AnchorConstraint
            continue
        elseif constraint isa XRollerConstraint
            push!(mapping, xi)
        elseif constraint isa YRollerConstraint
            push!(mapping, yi)
        else
            push!(mapping, xi)
            push!(mapping, yi)
        end
    end
    return mapping
end

"Construct a view of an array where constrained indices, indicated by a mapping, are removed. This is useful for solving matrices as zero rows will cause det(M)==0."
function constrained_array(mat::AbstractArray, mapping::Vector{<:Integer})
    return view(mat, mapping, mapping)
end
function constrained_array(vec::AbstractVector, mapping::Vector{<:Integer})
    return view(vec, mapping)
end

"Recover the original array where DOF indices will match with the StaticSetup."
original_array(arr::AbstractArray) = parent(arr)


function induced_forces(setup::StaticSetup, edge_id, displacements::Vector{<:Vector2D})
    k_local = member_stiffness_matrix(setup, edge_id) # N / m
    v1, v2 = terminal_joints(setup, edge_id)
    δx1, δy1 = displacements[v1]
    δx2, δy2 = displacements[v2]
    u_local = [δx1, δy1, δx2, δy2] # m
    fx1, fy1, fx2, fy2 = k_local * u_local # N / m * m -> N
    return (v1 => Vector2D(fx1, fy1), v2 => Vector2D(fx2, fy2))
end

"Get the weight, in Newtons, of a member by its member id."
weight(setup::StaticSetup, mid::Integer; kwargs...) = weight(setup.materials[mid], member_length(setup, mid); kwargs...)

weight(setup::StaticSetup; kwargs...) = sum((weight(setup, i; kwargs...) for i in member_ids(setup)))

"Get the mass, in kg, of a member by its member id."
mass(setup::StaticSetup, mid::Integer) = mass(setup.materials[mid], member_length(setup, mid))

mass(setup::StaticSetup) = sum((mass(setup, i) for i in member_ids(setup)))

"""
    add_member_weights!(setup::StaticSetup)

Add the weights that each ALL members exert on the setup as forces to their corresponding nodes.
- Note that nothing is checking if this function is being called multiple times, if you accidentally call this function twice, members will just act like they're twice as heavy. 
"""
function add_member_weights!(setup::StaticSetup)
    for mid in member_ids(setup)
        add_member_weights!(setup, mid)
    end
end


function add_member_weights!(setup::StaticSetup, mid::Integer)
    w = weight(setup, mid)
    joint_1, joint_2 = terminal_joints(setup, mid)
    add_force!(setup, joint_1, 0, -w/2)
    add_force!(setup, joint_2, 0, -w/2)
end