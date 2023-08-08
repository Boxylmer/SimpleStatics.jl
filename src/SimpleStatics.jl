module SimpleStatics
using Graphs
using LinearAlgebra

using Luxor


include("HelperFunctions.jl")
include("Base.jl")
export NoConstraint
export AnchorConstraint
export XRollerConstraint
export YRollerConstraint

export StaticMaterial
export Materials


export StaticSetup
export add_joint!
export add_member!
export set_force!
export set_constraint!
export global_stiffness_matrix

export n_joints
export n_members
export joint_ids
export member_ids



include("Solvers.jl")
export solve_displacements
export equilibrium_positions
export solve_member_stresses
export solve_reaction_forces

include("Visual.jl")
export plot_setup

end
