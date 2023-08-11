module SimpleStatics
using LinearAlgebra

using Luxor


include("HelperFunctions.jl")

include("StaticConstraint.jl")
export NoConstraint
export AnchorConstraint
export XRollerConstraint
export YRollerConstraint

include("StaticMaterial.jl")
export StaticMaterial
export Materials
export weight

include("StaticSetup.jl")
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
export member_angle

export add_member_weights!



include("Solvers.jl")
export solve_displacements
export equilibrium_positions
export solve_member_stresses
export solve_reaction_forces

include("Visual.jl")
export plot_setup

end
