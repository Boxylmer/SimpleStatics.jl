
abstract type StaticConstraint end

abstract type SimpleConstraint <: StaticConstraint end

struct NoConstraint <: SimpleConstraint end
struct AnchorConstraint <: SimpleConstraint end
struct XRollerConstraint <: SimpleConstraint end
struct YRollerConstraint <: SimpleConstraint end





struct StaticMaterial{AT, MT}
    area::AT # in square meters
    modulus::MT # in pa or netwon / meter^2
end

PerfectMaterial()::StaticMaterial = StaticMaterial(1.0, 1e12)


struct StaticsSetup{T}
    graph::SimpleGraph
    positions::Vector{<:Vector2D{T}} # Meters
    forces::Vector{<:Vector2D{T}} # newtons
    constraints::Vector{<:StaticConstraint}
    materials::Vector{<:StaticMaterial{T}}  # 
    vertices_to_edge::Dict{UnorderedPair, Int32}
    edge_to_vertices::Vector{<:UnorderedPair}  
end 

function StaticsSetup(T::Type)
    g = SimpleGraph()
    positions = Vector{Vector2D{T}}() # vertex id -> position of that joint
    forces = Vector{Vector2D{T}}() # vertex id -> force vector acting on that joint
    constraints = Vector{StaticConstraint}() # vertex id -> constraint for that joint
    materials = Vector{StaticMaterial{T}}() # edge id -> material for that member
    vertices_to_edge = Dict{UnorderedPair, Int32}() # use two vertices ids to look up the edge id connecting them
    edge_to_vertices = Vector{UnorderedPair}() # use an edge id to get the vertices it joins
    return StaticsSetup(g, positions, forces, constraints, materials, vertices_to_edge, edge_to_vertices)
end

Base.eltype(::StaticsSetup{T}) where T = T 

n_joints(s::StaticsSetup) = length(s.positions)
n_members(s::StaticsSetup) = length(s.materials)
joint_ids(s::StaticsSetup) = 1:n_joints(s)
member_ids(s::StaticsSetup) = 1:n_members(s)
n_dofs(s::StaticsSetup) = 2 * n_joints(s)


"Get the joints IDs that a member connects, found by its member ID."
terminal_joints(s::StaticsSetup, member_id::Integer) = s.edge_to_vertices[member_id]


initialxy(s::StaticsSetup, vid::Integer) = s.positions[vid].x, s.positions[vid].y
equilibriumxy(s::StaticsSetup, displacements::Vector{<:Vector2D}, vid::Integer) = s.positions[vid].x + displacements[vid].x, s.positions[vid].y + displacements[vid].y


"Add a joint to the setup and return the index referring to that joint. The position should be set in meters."
function add_joint!(setup::StaticsSetup{T}, position::Vector2D{T}, constraint::StaticConstraint=NoConstraint()) where T
    node_index = n_joints(setup) + 1
    add_vertex!(setup.graph)
    push!(setup.positions, position)
    push!(setup.constraints, constraint)
    push!(setup.forces, Vector2D(T(0), T(0)))
    return node_index
end

"Add a member to the setup and return the index referring to that member."
function add_member!(setup::StaticsSetup, node_index1::Int, node_index2::Int, material::StaticMaterial{T}=PerfectMaterial()) where T
    if node_index1 == node_index2
        throw(ArgumentError("A member cannot connect a joint to itself."))
    end
    
    edge_key = UnorderedPair(node_index1, node_index2)
    if haskey(setup.vertices_to_edge, edge_key)
        throw(ArgumentError("A member between these nodes already exists."))
    end
    
    add_edge!(setup.graph, node_index1, node_index2)
    push!(setup.materials, material)
    n = length(setup.materials)
    setup.vertices_to_edge[edge_key] = n
    push!(setup.edge_to_vertices, edge_key)
    return n
end

"Set the force at a joint, in Newtons."
function set_force!(setup::StaticsSetup{T}, joint_index::Int, force::Vector2D{T}) where T
    if typeof(setup.constraints[joint_index]) <: AnchorConstraint
        @warn "Force set on fully constrained joint! This won't cause anything to happen. Did you mean to place this force somewhere else?"
    end
    setup.forces[joint_index] = force
end

function set_constraint!(setup::StaticsSetup, joint_index::Int, constraint::StaticConstraint)
    setup.constraints[joint_index] = constraint
end

function member_length(setup::StaticsSetup, member_idx::Int)
    edge_key = setup.edge_to_vertices[member_idx]
    node_index1, node_index2 = edge_key.a, edge_key.b
            
    p1 = setup.positions[node_index1]
    p2 = setup.positions[node_index2]
    dx = p2.x - p1.x
    dy = p2.y - p1.y
            
    return sqrt(dx^2 + dy^2)
end

"Get the angle, in radians, the member makes with the global coordinate system's positive X axis."
function member_angle(setup::StaticsSetup, edge_id::Int)
    edge_key = setup.edge_to_vertices[edge_id]
    node_index1, node_index2 = edge_key

    p1 = setup.positions[node_index1]
    p2 = setup.positions[node_index2]
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    return atan(dy, dx)
end

"Get the global stiffness matrix for a StaticsSetup in Newtons / Meter."
function member_stiffness_matrix(setup::StaticsSetup, edge_id)
    l = member_length(setup, edge_id)
    edge_key = setup.edge_to_vertices[edge_id]
    node_index1, node_index2 = edge_key
    c = (setup.positions[node_index2].x - setup.positions[node_index1].x) / l
    s = (setup.positions[node_index2].y - setup.positions[node_index1].y) / l
    m = setup.materials[edge_id]
    A = m.area
    E = m.modulus
    scalar = A * E / l # units: newton / meter
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
function force_vector(setup::StaticsSetup)
    force_vec = zeros(eltype(setup), n_dofs(setup))
    for i in eachindex(setup.forces)
        force_vec[dof_indices(i)] .= setup.forces[i].x, setup.forces[i].y
    end
    return force_vec
end

function global_stiffness_matrix(setup::StaticsSetup)
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

"Recover the original array where DOF indices will match with the StaticsSetup."
original_array(arr::AbstractArray) = parent(arr)