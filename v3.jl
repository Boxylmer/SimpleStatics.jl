using Graphs
using LinearAlgebra

using Test

"Create a pair of items whose order is independent in hashes and comparisons."
struct UnorderedPair; a::Int32; b::Int32; end
Base.hash(x::UnorderedPair, h::UInt) = hash(x.a < x.b ? (x.a, x.b) : (x.b, x.a), h)
Base.isequal(x::UnorderedPair, y::UnorderedPair) = x.a == y.a ? x.b == y.b : (x.a == y.b && x.b == y.a)
higher(x::UnorderedPair) = max(x.a, x.b)
lower(x::UnorderedPair) = min(x.a, x.b)
Base.:(==)(x::UnorderedPair, y::UnorderedPair) = isequal(x, y)
function Base.iterate(pair::UnorderedPair, state=1)
    if state == 1
        return (pair.a, state+1)
    elseif state == 2
        return (pair.b, nothing)
    end
end
Base.length(::UnorderedPair) = 2


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
# Addition
Base.:+(u::Vector2D, v::Vector2D) = Vector2D(u.x + v.x, u.y + v.y)

# Subtraction
Base.:-(u::Vector2D, v::Vector2D) = Vector2D(u.x - v.x, u.y - v.y)

# Scalar multiplication (left and right)
Base.:*(a, v::Vector2D) = Vector2D(a * v.x, a * v.y)
Base.:*(v::Vector2D, a) = Vector2D(v.x * a, v.y * a)

function Base.iterate(v::Vector2D, state=1)
    if state == 1
        return (v.x, state+1)
    elseif state == 2
        return (v.y, nothing)
    end
end
Base.length(::Vector2D) = 2

norm(v::Vector2D) = sqrt(v.x^2 + v.y^2)

function unit_vector(v::Vector2D)
    mag = sqrt(v.x^2 + v.y^2)
    return Vector2D(v.x / mag, v.y / mag)
end

Base.eltype(::Vector2D{T}) where T = T 

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


function solve_displacements(setup::StaticsSetup{T}, gsm::Matrix{T}) where T
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
solve_displacements(setup::StaticsSetup) = solve_displacements(setup, global_stiffness_matrix(setup))

equilibrium_positions(s::StaticsSetup, displacements::Vector{<:Vector2D}) = s.positions .+ displacements

"Negative -> Compressive, Positive -> Tensile, index matches member index"
function solve_member_stresses(setup::StaticsSetup{T}, displacements::Vector{<:Vector2D}) where T
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



### plotting
using Luxor

function find_domain(vals::AbstractArray{<:Number}, padding=0)
min_x, max_x = vals[1], vals[1]
    for v in vals
        if v > max_x; max_x = v
        elseif v < min_x; min_x = v
        end
    end
    xpad = padding * (max_x - min_x)
    return (min_x - xpad,  max_x + xpad)
end

function find_domain(points::Vector{<:Vector2D}, padding=0)
    min_x, max_x, min_y, max_y = points[1].x, points[1].x, points[1].y, points[1].y
    for pt in points
        if pt.x > max_x; max_x = pt.x
        elseif pt.x < min_x; min_x = pt.x
        end
        if pt.y > max_y; max_y = pt.y
        elseif pt.y < min_y; min_y = pt.y
        end
    end
    xpad = padding * (max_x - min_x)
    ypad = padding * (max_y - min_y)
    return (min_x - xpad,  max_x + xpad), (min_y - ypad,  max_y + ypad)
end

function transform_value(x, xbounds_original, xbounds_final)
    return ((xbounds_final[2] - xbounds_final[1]) / (xbounds_original[2] - xbounds_original[1])) * (x - xbounds_original[1]) + xbounds_final[1]
end

function transform_for_luxor(pts::Vector{<:Vector2D}, canvas_size, xdomain, ydomain)
    canvas_x_domain = (-canvas_size[1] / 2, canvas_size[1] / 2)
    canvas_y_domain = (canvas_size[2] / 2, -canvas_size[2] / 2)
    luxor_pts = Vector{Point}(undef, length(pts))
    for (i, pt) in enumerate(pts)
        luxor_pts[i] = Point(
            transform_value(pt.x, xdomain, canvas_x_domain),
            transform_value(pt.y, ydomain, canvas_y_domain)
        )
    end
    return luxor_pts
end

function transform_for_luxor(pts::Vector{<:Vector2D}, canvas_size, padding=0)
    xdomain, ydomain = find_domain(pts, padding)
    return transform_for_luxor(pts, canvas_size, xdomain, ydomain)
end

function redistribute_linearly(values, desired_average)
        non_zero_vals = filter(x -> !(x ≈ 0), values)
        average = sum(non_zero_vals) / length(non_zero_vals)
        alpha = desired_average / average
        T = [alpha * v for v in values]
        return T
end

function plot_setup(s::StaticsSetup, name="default"; dsize=800, padding=0.4, displacements=nothing, stresses=nothing)
    xdomain, ydomain = find_domain(s.positions, padding)
    xlen = xdomain[2] - xdomain[1]
    ylen = ydomain[2] - ydomain[1]
    
    mag = sqrt(xlen^2 + ylen^2)
    
    xunit = xlen / mag
    yunit = ylen / mag
    
    size = (xunit * dsize, yunit * dsize)
    pts = transform_for_luxor(s.positions, size, xdomain, ydomain)

    # @svg begin
    Drawing(size[1], size[2], :svg, name * ".svg")
        background("antiquewhite")
        ### Joints
        sethue("black")
        for (i, p) in enumerate(pts)
            circle(p, 2, action = :fill)

            label("P"*string(i), :NE, p)

        end
        
        sethue("black")
        ### Members
        for mid in member_ids(s)
            i, j = s.edge_to_vertices[mid]
            p1 = pts[i]
            p2 = pts[j]
            line(p1, p2, action = :stroke)
            label("M"*string(mid), :NE, midpoint(p1, p2))
        end
        
        ### displacements
        if !isnothing(displacements)
            sethue("green")
            displaced_pts = transform_for_luxor(s.positions .+ displacements, size, xdomain, ydomain)

            for (i, p) in enumerate(pts)
                circle(displaced_pts[i], 2, action = :fill)
            end

            ### Stresses 
            function draw_member(mid)
                i, j = s.edge_to_vertices[mid]
                p1 = displaced_pts[i]
                p2 = displaced_pts[j]
                line(p1, p2, action = :stroke)
            end

            if isnothing(stresses)
                for mid in member_ids(s)
                    draw_member(mid)
                end
            else
                sd = find_domain(stresses)
                for mid in member_ids(s)
                    
                    stress = stresses[mid]

                    if stress > 0
                        color = (stress / sd[2], 0, 0)
                    else
                        color = (0, 0, stress / sd[1])
                    end
                    
                    sethue(color)
                    draw_member(mid)
                end
            end
        end
        

        ### Forces
        sethue("mediumvioletred")
        nonzero_force_indices = [i for i in eachindex(s.forces) if (s.forces[i].x != 0 || s.forces[i].y != 0)]
        nonzero_force_vectors = [s.forces[i] for i in nonzero_force_indices]
        nonzero_force_magnitudes = norm.(nonzero_force_vectors)
        average_relative_force_length = 0.1
        nonzero_force_pixel_lengths = redistribute_linearly(
            nonzero_force_magnitudes, 
            average_relative_force_length * sqrt(size[1]^2 + size[2]^2)
        )
        nonzero_force_unit_vectors = [Point(v.x, -v.y) for v in unit_vector.(nonzero_force_vectors)]
        @show nonzero_force_pixel_vectors = nonzero_force_pixel_lengths .* nonzero_force_unit_vectors

        for (i, j) in enumerate(nonzero_force_indices)
            p1 = pts[j]
            p2 = pts[j] + nonzero_force_pixel_vectors[i]
            arrow(p1, p2)
            label("F"*string(i), :N, midpoint(p1, p2))
        end

        ### constraints

    # end size[1] size[2]
    finish()
    preview()
end



### examples



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


function example_setup_2()
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
    set_constraint!(s, j4, XRollerConstraint())
    set_force!(s, j3, Vector2D(5000., -3000.))
    return s
end

s2 = example_setup_2()
plot_setup(s2, "2 - Setup")
global_stiffness_matrix(s2) 
d2 = solve_displacements(s2)
st2 = solve_member_stresses(s2, d2)
plot_setup(s2, "2 - Solution", stresses=st2, displacements=d2)


function example_setup_3()
    s = StaticsSetup(Float64)
    j1 = add_joint!(s, Vector2D(0.0, 0.0))
    j2 = add_joint!(s, Vector2D(16.0 * 12, 0.0 * 12))
    j3 = add_joint!(s, Vector2D(16.0 * 12, 12.0 * 12))
    j4 = add_joint!(s, Vector2D(32.0 * 12, 0.0 * 12))
    j5 = add_joint!(s, Vector2D(32.0 * 12, 12.0 * 12))

    m = StaticMaterial(10., 300)
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
    set_force!(s, j2, Vector2D(0., 50.))
    set_force!(s, j5, Vector2D(-50., 0.))
    return s
end

s3 = example_setup_3()
member_stiffness_matrix(s3, 1)
member_stiffness_matrix(s3, 2)
member_stiffness_matrix(s3, 3)
member_stiffness_matrix(s3, 4)
member_stiffness_matrix(s3, 5)
member_stiffness_matrix(s3, 6)
member_stiffness_matrix(s3, 7)
member_stiffness_matrix(s3, 8)

plot_setup(s3, "3 - Setup"; dsize=1000)
gsm3 = global_stiffness_matrix(s3)
d3 = solve_displacements(s3) 
st3 = solve_member_stresses(s3, d3)
plot_setup(s3, "3 - Solution"; displacements=d3, stresses=st3, dsize=1000)



function example_setup_4()
    # Matrix Analysis of Structures, chapter 4, truss in example 4.2
    s = StaticsSetup(Float64)
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