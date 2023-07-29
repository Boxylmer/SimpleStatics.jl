using Graphs

abstract type StaticConstraint end

struct SimpleConstraint <: StaticConstraint
    x::Int16 # 1 if x adds a degree of freedom (it can move). 0 if it's locked in place. 
    y::Int16
end

XRollerConstraint() = SimpleConstraint(1, 0)
YRollerConstraint() = SimpleConstraint(0, 1)
PinConstraint() = SimpleConstraint(1, 1)
AnchorConstraint() = SimpleConstraint(0, 0)

struct Vector2D{T}
    x::T
    y::T
end

struct StaticMaterial{AT, MT}
    area::AT
    modulus::MT
end

struct StaticsSetup{PT, FT, MT}
    graph::SimpleGraph
    positions::Vector{<:Vector2D{PT}}
    forces::Vector{Vector2D{FT}}
    constraints::Vector{StaticConstraint}
    materials::Vector{StaticMaterial{MT}}
end 


# working tests

g = SimpleGraph()
add_vertex!(g)  # index will be one
add_vertex!(g)  # index will be two
add_vertex!(g)  # etc...
add_vertex!(g)

add_edge!(g, 1, 2)
add_edge!(g, 1, 3)
add_edge!(g, 2, 3)
add_edge!(g, 2, 4)
add_edge!(g, 3, 4)
using Plots
using GraphRecipes
Graphplot(g, curves=false)

neighbors(g, 1)

adjacency_matrix(g)




function local_stiffness_matrix(i::Integer)
end
