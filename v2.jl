using Graphs
using LinearAlgebra

"Create a pair of items whose order is independent in hashes and comparisons."
struct UnorderedPair; a::Int32; b::Int32; end
Base.hash(x::UnorderedPair, h::UInt) = hash(x.a < x.b ? (x.a, x.b) : (x.b, x.a), h)
Base.isequal(x::UnorderedPair, y::UnorderedPair) = x.a == y.a ? x.b == y.b : (x.a == y.b && x.b == y.a)
higher(x::UnorderedPair) = max(x.a, x.b)
lower(x::UnorderedPair) = min(x.a, x.b)
Base.:(==)(x::UnorderedPair, y::UnorderedPair) = isequal(x, y)

abstract type StaticConstraint end

abstract type SimpleConstraint <: StaticConstraint end

struct NoConstraint <: SimpleConstraint end
struct AnchorConstraint <: SimpleConstraint end
struct XRollerConstraint <: SimpleConstraint end
struct YRollerConstraint <: SimpleConstraint end

struct Vector2D{T}
    x::T
    y::T
end

Base.eltype(::Vector2D{T}) where T = T 

struct StaticMaterial{AT, MT}
    area::AT # in square meters
    modulus::MT # in pa or netwon / meter^2
end

PerfectMaterial()::StaticMaterial = StaticMaterial(1.0, 1e4)


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

"Add a joint to the setup and return the index referring to that joint. The position should be set in meters."
function add_joint!(setup::StaticsSetup{T}, position::Vector2D{T}, constraint::StaticConstraint=NoConstraint()) where T
    node_index = nv(setup.graph) + 1
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
set_constraint!(s, j4, YRollerConstraint())
set_force!(s, j3, Vector2D(000., 50000.))

# s = StaticsSetup(Float64)
# j1 = add_joint!(s, Vector2D(0.0, 0.0))
# j2 = add_joint!(s, Vector2D(1.0, 1.0))
# j3 = add_joint!(s, Vector2D(2.0, 0.0))

# m1 = add_member!(s, j1, j2)
# m2 = add_member!(s, j2, j3)
# m3 = add_member!(s, j3, j1)

# set_constraint!(s, j1, AnchorConstraint())
# set_constraint!(s, j3, XRollerConstraint())
# set_force!(s, j2, Vector2D(2.3, -40.3))



# test out some of graph.jl stuff
neighbors(s.graph, 1)
adjacency_matrix(s.graph)



function member_length(setup::StaticsSetup, member_idx::Int)
    edge_key = setup.edge_to_vertices[member_idx]
    node_index1, node_index2 = edge_key.a, edge_key.b
            
    p1 = setup.positions[node_index1]
    p2 = setup.positions[node_index2]
    dx = p2.x - p1.x
    dy = p2.y - p1.y
            
    return sqrt(dx^2 + dy^2)
end

using Test
@test member_length(s, m1) == 1
@test member_length(s, m2) == sqrt(2)
@test member_length(s, m3) == 1
@test member_length(s, m4) == 1
@test member_length(s, m5) == sqrt(2)

"Get the angle, in radians, the member makes with the global coordinate system's positive X axis."
function member_angle(setup::StaticsSetup, edge_id::Int)
    edge_key = setup.edge_to_vertices[edge_id]
    node_index1, node_index2 = edge_key.a, edge_key.b

    p1 = setup.positions[node_index1]
    p2 = setup.positions[node_index2]
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    return atan(dy, dx)
end

@test rad2deg(member_angle(s, m1)) == 0
@test rad2deg(member_angle(s, m2)) == 45
@test rad2deg(member_angle(s, m3)) == 90
@test rad2deg(member_angle(s, m4)) == 0
@test rad2deg(member_angle(s, m5)) == -45.0

function local_to_global_transformation_matrix(angle::Real)
    c = cos(angle)
    s = sin(angle)
    # T = [
    #     c -s 0 0; 
    #     s c 0 0; 
    #     0 0 c -s; 
    #     0 0 s c
    # ]
    T = [
        c s; 
       -s c;
    ]
    return T
end

function local_to_global_transformation_matrix(setup::StaticsSetup, edge_id::Int)
    a = member_angle(setup, edge_id)
    return local_to_global_transformation_matrix(a)
end

# @test local_to_global_transformation_matrix(s, m1) == [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]
@test local_to_global_transformation_matrix(s, m1) == [1 0; 0 1;]
@test local_to_global_transformation_matrix(s, m2) == local_to_global_transformation_matrix(member_angle(s, m2))



"Get the local stiffness matrix for a member in Newtons / Meter."
function local_stiffness_matrix(material::StaticMaterial, length::Real)
    A = material.area
    E = material.modulus
    k = A * E / length # units: newton / meter
    k_local = [k -k; -k k]
    return k_local
end

"Get the local stiffness matrix for a member in Newtons / Meter."
function local_stiffness_matrix(setup::StaticsSetup, edge_id::Integer)
    l = member_length(setup, edge_id)
    m = setup.materials[edge_id]
    return local_stiffness_matrix(m, l)
end

@test local_stiffness_matrix(s, m1) == local_stiffness_matrix(PerfectMaterial(), 1)
local_stiffness_matrix(s, m2)

"Get the global stiffness matrix for a StaticsSetup in Newtons / Meter."
function global_stiffness_matrix(setup::StaticsSetup, edge_id::Integer)
    k_local = local_stiffness_matrix(setup, edge_id)
    T = local_to_global_transformation_matrix(setup, edge_id)
    T' * k_local * T
end

global_stiffness_matrix(s, m2)
global_stiffness_matrix(s, m3)
global_stiffness_matrix(s, m4)
global_stiffness_matrix(s, m5)

"Get the indices of the x and y rows, respectively, in a square DOF-like matrix (e.g., global stiffness matrix) for a vertex (joint) index."
function dof_indices(node_index::Integer)
    row_start = 2 * node_index - 1
    row_end = 2 * node_index
    return row_start:row_end
end

"Get a vector of the unconstrained forces in the setup."
function force_vector(setup::StaticsSetup)
    n_nodes = nv(setup.graph)
    n_dofs = 2 * n_nodes
    force_vec = zeros(eltype(setup), n_dofs)
    for i in eachindex(setup.forces)
        force_vec[dof_indices(i)] .= setup.forces[i].x, setup.forces[i].y
    end
    return force_vec
end

function global_stiffness_matrix(setup::StaticsSetup)
    n_nodes = nv(setup.graph)
    n_dofs = 2 * n_nodes
    k_global = zeros(n_dofs, n_dofs)

    for edge_id in 1:length(setup.edge_to_vertices)
        K_member = global_stiffness_matrix(setup, edge_id)

        # Get the DOF indices for the nodes of the current member
        edge_key = setup.edge_to_vertices[edge_id]
        node_index1, node_index2 = edge_key.a, edge_key.b
        node_index1_indices = dof_indices(node_index1)
        node_index2_indices = dof_indices(node_index2)

        # Add the member's stiffness matrix to the global stiffness matrix
        for i in 1:2
            for j in 1:2
                row_i = node_index1_indices[i]
                col_j = node_index1_indices[j]
                k_global[row_i, col_j] += K_member[i, j]

                row_i = node_index2_indices[i]
                col_j = node_index2_indices[j]
                k_global[row_i, col_j] += K_member[i, j]
            end
        end

    end

    return k_global
end

function example_setup_1()
    # Matrix Analysis of Structures, chapter 3, example 3.7
    s = StaticsSetup(Float64)
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
    set_force!(s, j1, Vector2D(1000., 0.))
    return s
end

s1 = example_setup_1()
global_stiffness_matrix(s1) 

gsm = global_stiffness_matrix(s)

"Set the indices of the global stiffness matrix that are constrained to zero."
function zero_constrained_indices!(setup, gsm = global_stiffness_matrix(setup))
    n_nodes = nv(setup.graph)
    for node_index in 1:n_nodes
        constraint = setup.constraints[node_index]
        if constraint isa AnchorConstraint
            # Joint is fully constrained (both x and y directions are locked)
            # Zero out rows and columns in the global stiffness matrix
            row_indices = dof_indices(node_index)
            gsm[row_indices, :] .= 0
            gsm[:, row_indices] .= 0
        elseif constraint isa XRollerConstraint
            # Joint is constrained to the x-direction (horizontal roller)
            # Zero out the row corresponding to y motion in the global stiffness matrix
            row_indices = dof_indices(node_index)[2]
            gsm[row_indices, :] .= 0
            gsm[:, row_indices] .= 0
        elseif constraint isa YRollerConstraint
            # Joint is constrained to the y-direction (vertical roller)
            # Zero out the row corresponding to x motion in the global stiffness matrix
            row_indices = dof_indices(node_index)[1]
            gsm[row_indices, :] .= 0
            gsm[:, row_indices] .= 0
        end
    end

    return gsm
end

"Find a mapping m[i] -> j where i is the index of a constrained dof array and j is the corresponding index of the unconstrained array."
function _constrained_to_original_dof_array_mapping(setup)
    n_nodes = nv(setup.graph)
    mapping = Vector{Int64}() 

    for node_index in 1:n_nodes
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

_constrained_to_original_dof_array_mapping(s)

"Construct an array where constrained indices, indicated by a mapping, are removed."
function constrained_array(mat::Matrix, mapping::Vector{Int})
    n_unconstrained = length(mapping)
    constrained = zeros(eltype(mat), n_unconstrained, n_unconstrained)

    for (i, j) in enumerate(mapping)
        for (k, l) in enumerate(mapping)
            constrained[i, k] = mat[j, l]
        end
    end

    return constrained
end

"Construct an array where constrained indices, indicated by a mapping, are removed."
function constrained_array(vec::AbstractVector, mapping::Vector{Int})
    n_unconstrained = length(mapping)
    constrained = zeros(eltype(vec), n_unconstrained)
    
    for (i, j) in enumerate(mapping)
        constrained[i] = vec[j]
    end

    return constrained
end

"Construct a global stiffness matrix where the constrained indices are removed, using a precomputed mapping if available."
function constrained_global_stiffness_matrix(setup, global_stiffness_matrix, mapping=_constrained_to_original_dof_array_mapping(setup))
    return constrained_array(global_stiffness_matrix, mapping)
end

constrained_global_stiffness_matrix(s, gsm)

"Construct a force vector where the constrained indices are removed, using a precomputed mapping if available."
function constrained_force_vector(setup::StaticsSetup, m = _constrained_to_original_dof_array_mapping(setup)) 
    return constrained_array(force_vector(setup), m)
end

constrained_force_vector(f::AbstractVector{<:Real}, m) = constrained_array(f, m)

@test constrained_force_vector(s) == constrained_force_vector(force_vector(s), _constrained_to_original_dof_array_mapping(s))


function solve_displacements(setup::StaticsSetup, gsm = global_stiffness_matrix(setup))
    # setup.forces' index corresponds to the vertex id and contains a Vector2D
    force_vec = force_vector(setup)
    m = _constrained_to_original_dof_array_mapping(setup)
    @show cfv = constrained_force_vector(force_vec, m)
    @show cgsm = constrained_global_stiffness_matrix(setup, gsm, m)
    displacements = cfv \ cgsm # cgsm \ cfv <- ignore  # units:  N / (N / m) -> m 

    return displacements
end

solve_displacements(setup::StaticsSetup) = solve_displacements(setup, global_stiffness_matrix(setup))

@show solve_displacements(s) 


function example_setup_1()
    # Matrix Analysis of Structures, chapter 3, example 3.7
    s = StaticsSetup(Float64)
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
    set_force!(s, j1, Vector2D(1000., 0.))
    return s
end

s1 = example_setup_1()
global_stiffness_matrix(s1) 
solve_displacements(s1) #  20.58m  0.0m  0.0m 
