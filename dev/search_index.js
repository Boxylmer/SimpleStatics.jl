var documenterSearchIndex = {"docs":
[{"location":"example workflow/#Example-Workflow","page":"Example Workflow","title":"Example Workflow","text":"","category":"section"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"As the name might suggest, structures can be created and analyzed with fair ease, with each problem being started with","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"using SimpleStatics\r\nmy_setup = StaticSetup()","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"creating an empty statics problem which can then be filled with members, joints, forces, etc.","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Lets create a simple truss to analyze. ","category":"page"},{"location":"example workflow/#.-Add-Joints","page":"Example Workflow","title":"1. Add Joints","text":"","category":"section"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"First, we need to add some joints. Joints act as terminals to members, which we will add in the next step. They also need constraints to dictate their movement behavior. By default, they are assigned NoConstraint(). For our first joint, which we want to keep fixed in place, we'll give it an AnchorConstraint().","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"We can add joints using the add_joint!(setup, x, y, constraint) method, which will return the index of the joint that was created. In this case, since we don't have too many joints to add, we can simply assign them to individual variables.","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"tip: Visualizing your progress\nYou can look at what you've created at any point by calling the plot_setup(::StaticSetup) function on your setup! Below we will use this to see what we've created in our example workflow. The use of this function is shown in step 1, but is hidden in future steps. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"j1 = add_joint!(my_setup, 0, 0, AnchorConstraint())\r\nj2 = add_joint!(my_setup, 1, 0)\r\nj3 = add_joint!(my_setup, 1, 1)\r\nj4 = add_joint!(my_setup, 2, 0)\r\nj5 = add_joint!(my_setup, 2, 1)\r\nj6 = add_joint!(my_setup, 3, 0)\r\nj7 = add_joint!(my_setup, 3, 1)\r\nj8 = add_joint!(my_setup, 4, 0)\r\n\r\nplot_setup(my_setup)","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Notice the following","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Joint 1 was given an AnchorConstraint() which will keep it fixed in place. \nPoints were automatically labeled in the order they were created. \nConstraints were depicted in green denoting the kind of motion allowed for the joint. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"If we want to modify constraints after the joint was already added, we can do so using the set_constraint! function. Lets add an XRollerConstraint() to J8, as we want it to stay fixed on the Y axis, but be allowed to move along the X axis.","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"set_constraint!(my_setup, j8, XRollerConstraint())\r\n\r\nplot_setup(my_setup) # hide","category":"page"},{"location":"example workflow/#.-Add-Members","page":"Example Workflow","title":"2. Add Members","text":"","category":"section"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Now we need to connect our joints via members. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Similar to how a joint 's behavior is controlled by its constraint, a member's behavior is controlled by its material, these are accessed via the Materials submodule. Members have a default for this as well: A PerfectMaterial(), which is a very thick material with a tensile strength a few orders higher than that of Tungsten, the strongest rote metal. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Lets use standard 2x4 lumber for our truss structure.","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"m = Materials.Pine2x4()","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"We can now add members using the add_member!(setup, j1, j2, material) function. This function will also return an identifier for each member that could be used to refer to them later on, but since we don't need to do anything directly to them after this, we don't need to assign them to anything. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Lets set up a more proper-looking truss. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"add_member!(my_setup, j1, j2, m)\r\nadd_member!(my_setup, j1, j3, m)\r\nadd_member!(my_setup, j2, j3, m)\r\nadd_member!(my_setup, j2, j4, m)\r\nadd_member!(my_setup, j3, j4, m)\r\nadd_member!(my_setup, j3, j5, m)\r\nadd_member!(my_setup, j4, j5, m)\r\nadd_member!(my_setup, j4, j6, m)\r\nadd_member!(my_setup, j5, j6, m)\r\nadd_member!(my_setup, j5, j7, m)\r\nadd_member!(my_setup, j6, j7, m)\r\nadd_member!(my_setup, j6, j8, m)\r\nadd_member!(my_setup, j7, j8, m)\r\nplot_setup(my_setup) # hide","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Whew! That was a lot of members. This can be tedious, but it's a necessary step in creating our masterpiece. Looking at the result immediately shows us the fruits of our labor, as we now have a very nice looking structure. ","category":"page"},{"location":"example workflow/#.-Add-Forces","page":"Example Workflow","title":"3. Add Forces","text":"","category":"section"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"This truss isn't very useful to us unless we understand more about how it handles loads! ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"We can add forces to joints by using the set_force!(setup, j, fx, fy) function. Lets add a 50lbf (222411N) load directly on the top of the truss, which would be at J5 in the -Y direction. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"set_force!(my_setup, j5, 0, -222.411)\r\nplot_setup(my_setup) # hide","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Now we can see a very nice pink arrow depicting our force and how it's acting on our structure!","category":"page"},{"location":"example workflow/#.-Calculate-Displacements","page":"Example Workflow","title":"4. Calculate Displacements","text":"","category":"section"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"After completing step 3, we've fully defined a statics problem. Now the question is: What can we actually do with it? The first question is generally centered around how the forces acting on the truss deforms it, which we can easily calculate using the solve_displacements(setup) function!","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"tip: Joint indices match displacements\nThe displacements we get out of this funciton will follow the same order as the joints we created, so we can see the displacement that happened to J5, we simply access it in the calculated displacements using j5. We can also access each component of the displacements via displacement[idx].x or -.y if needed. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"displacements = solve_displacements(my_setup)\r\ndisplacements[j5]","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"It looks like J5 moved down by about 34times10^-5m or 000137. This certainly isn't a lot. Let see what happened visually as well to confirm this for all of our points!","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"This time our plot_setup function will need to have the displacements data as well to be able to show us what actually happened. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"plot_setup(my_setup, displacements=displacements)","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Members are now displayed in green to show us the displaced structure, and sure enough, none of the points really moved. Lets try adding a few thousand more pounds to J5 and seeing what happens.","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"set_force!(my_setup, j5, 0, -22241.11) # about 5000 pounds force. \r\ndisplacements = solve_displacements(my_setup)\r\ndisplacements[j5]","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"plot_setup(my_setup, displacements=displacements) # hide","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Hmm... we're applying 5k lbf and only seeing about 017 of sag? That doesn't sound right. Surely 2x4 lumber isn't that strong? There's good news and bad news here. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Good news: The 2x4 lumber you built your truss with is absolutely that strong.\nBad news: The screws you used to hold them together were not. ","category":"page"},{"location":"example workflow/#.-Calculate-Member-Stresses","page":"Example Workflow","title":"5. Calculate Member Stresses","text":"","category":"section"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Okay, so how do we solve this?  Let's say a standard deck screw can hold 100lbf (444822N)  comfortably before it shears, and we use two screws per board, getting about 850N (conservatively) of force that each board (member) can handle before it breaks. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"We can calculate the stress each board is under with the solve_member_stresses(setup, displacements) function. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"tip: Remember, plot_setup is your friend!\nWe can also pass the result of solve_member_stresses into our plot_setup function using the stresses keyword! Though these will only be visible if displacements are also present. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"member_stresses = solve_member_stresses(my_setup, displacements)\r\nmaximum(member_stresses), minimum(member_stresses) # in Newtons","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"First and foremost we notice that, as expected, our truss will be destroyed ten times over given the maximum stress on the members. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"note: Sign conventions are important!\nConventionally, negative stresses indicate tension and positive stresses indicate compression. This signage is arbitrary, but it's what was chosen for this library.","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Lets switch back to our 50lbf setup and check the maximum member force to see if we can at least bear that load without the risk of failure.","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"set_force!(my_setup, j5, 0, -222.411) # about 50 pounds force. \r\ndisplacements = solve_displacements(my_setup)\r\nmember_stresses = solve_member_stresses(my_setup, displacements)","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"plot_setup(my_setup, displacements=displacements, stresses=member_stresses) # hide","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Now we have some interesting results! Lets go over them, noting that compressive forces are shown in red and tensile forces in blue. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Notice that M3 and M11 are so-called zero force members. These members technically don't bear any load, but are necessary to constrain the system of equations that our setup has generated.\nM6 and M8 have equal and opposite stresses equal to the load on J5, whereas M9 and M10, as well as M4 and M5 are much lower, due to the load being distributed (unevenly) between the two. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"What will happen if we modify our truss slightly by recreating it with a member from J4 to J7 rather than J5 to J6. ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"    my_setup = StaticSetup()\r\n    j1 = add_joint!(my_setup, 0, 0, AnchorConstraint())\r\n    # ...\r\n    j2 = add_joint!(my_setup, 1, 0) # hide\r\n    j3 = add_joint!(my_setup, 1, 1) # hide\r\n    j4 = add_joint!(my_setup, 2, 0) # hide\r\n    j5 = add_joint!(my_setup, 2, 1) # hide\r\n    j6 = add_joint!(my_setup, 3, 0) # hide\r\n    j7 = add_joint!(my_setup, 3, 1) # hide\r\n    j8 = add_joint!(my_setup, 4, 0, XRollerConstraint()) # hide\r\n    add_member!(my_setup, j1, j2, m) # hide\r\n    add_member!(my_setup, j1, j3, m) # hide\r\n    add_member!(my_setup, j2, j3, m) # hide\r\n    add_member!(my_setup, j2, j4, m) # hide\r\n    add_member!(my_setup, j3, j4, m) # hide\r\n    add_member!(my_setup, j3, j5, m) # hide\r\n    add_member!(my_setup, j4, j5, m) # hide\r\n    add_member!(my_setup, j4, j6, m) # hide\r\n    add_member!(my_setup, j4, j7, m) # hide\r\n    add_member!(my_setup, j5, j7, m) # hide\r\n    add_member!(my_setup, j6, j7, m) # hide\r\n    add_member!(my_setup, j6, j8, m) # hide\r\n    add_member!(my_setup, j7, j8, m) # hide\r\n    set_force!(my_setup, j5, 0, -222.411)\r\n    displacements = solve_displacements(my_setup)\r\n    member_stresses = solve_member_stresses(my_setup, displacements)","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"plot_setup(my_setup, displacements=displacements, stresses=member_stresses) # hide","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"Apparently nothing good, as now one extra member is under a significantly higher load. What if we add a few extra cross members? ","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"    add_member!(my_setup, j2, j5, m) \r\n    add_member!(my_setup, j5, j6, m) \r\n    displacements = solve_displacements(my_setup)\r\n    member_stresses = solve_member_stresses(my_setup, displacements)\r\n    maximum(member_stresses), minimum(member_stresses)","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"plot_setup(my_setup, displacements=displacements, stresses=member_stresses) # hide","category":"page"},{"location":"example workflow/","page":"Example Workflow","title":"Example Workflow","text":"The result is a bit messy, but it looks like the maximum member stress was reduced!","category":"page"},{"location":"examples/example1/#Example-1-Mild-Steel-Truss","page":"Example 1","title":"Example 1 - Mild Steel Truss","text":"","category":"section"},{"location":"examples/example1/","page":"Example 1","title":"Example 1","text":"using SimpleStatics\r\n\r\n","category":"page"},{"location":"reference/","page":"Index","title":"Index","text":"","category":"page"},{"location":"reference/","page":"Index","title":"Index","text":"Modules = [SimpleStatics]","category":"page"},{"location":"reference/#SimpleStatics.UnorderedPair","page":"Index","title":"SimpleStatics.UnorderedPair","text":"Create a pair of items whose order is independent in hashes, equality, and comparisons.\n\n\n\n\n\n","category":"type"},{"location":"reference/#SimpleStatics.add_joint!-Union{Tuple{T}, Tuple{StaticSetup{T}, SimpleStatics.Vector2D{T}}, Tuple{StaticSetup{T}, SimpleStatics.Vector2D{T}, SimpleStatics.StaticConstraint}} where T","page":"Index","title":"SimpleStatics.add_joint!","text":"Add a joint to the setup and return the index referring to that joint. The position should be set in meters.\n\n\n\n\n\n","category":"method"},{"location":"reference/#SimpleStatics.add_member!-Union{Tuple{T}, Tuple{StaticSetup, Int64, Int64}, Tuple{StaticSetup, Int64, Int64, StaticMaterial{T}}} where T","page":"Index","title":"SimpleStatics.add_member!","text":"Add a member to the setup and return the index referring to that member.\n\n\n\n\n\n","category":"method"},{"location":"reference/#SimpleStatics.constrained_array-Tuple{AbstractArray, Vector{<:Integer}}","page":"Index","title":"SimpleStatics.constrained_array","text":"Construct a view of an array where constrained indices, indicated by a mapping, are removed. This is useful for solving matrices as zero rows will cause det(M)==0.\n\n\n\n\n\n","category":"method"},{"location":"reference/#SimpleStatics.constrained_dof_array_mapping-Tuple{Any}","page":"Index","title":"SimpleStatics.constrained_dof_array_mapping","text":"Find a mapping m[i] -> j where i is the index of a constrained dof array and j is the corresponding index of the unconstrained array.\n\n\n\n\n\n","category":"method"},{"location":"reference/#SimpleStatics.dof_indices-Tuple{Integer}","page":"Index","title":"SimpleStatics.dof_indices","text":"Get the indices of the x and y rows, respectively, in a square DOF-like matrix (e.g., global stiffness matrix) for a vertex (joint) index.\n\n\n\n\n\n","category":"method"},{"location":"reference/#SimpleStatics.force_vector-Tuple{StaticSetup}","page":"Index","title":"SimpleStatics.force_vector","text":"Get a vector of the unconstrained forces in the setup.\n\n\n\n\n\n","category":"method"},{"location":"reference/#SimpleStatics.global_stiffness_matrix-Tuple{StaticSetup}","page":"Index","title":"SimpleStatics.global_stiffness_matrix","text":"Get the stiffness matrix of the entire setup with respect to the global coordinate system.\n\n\n\n\n\n","category":"method"},{"location":"reference/#SimpleStatics.member_angle-Tuple{StaticSetup, Int64}","page":"Index","title":"SimpleStatics.member_angle","text":"Get the angle, in radians, the member makes with the global coordinate system's positive X axis.\n\n\n\n\n\n","category":"method"},{"location":"reference/#SimpleStatics.member_stiffness_matrix-Tuple{StaticSetup, Any}","page":"Index","title":"SimpleStatics.member_stiffness_matrix","text":"Get the global stiffness matrix for a StaticSetup in Newtons / Meter.\n\n\n\n\n\n","category":"method"},{"location":"reference/#SimpleStatics.n_dofs-Tuple{StaticSetup}","page":"Index","title":"SimpleStatics.n_dofs","text":"Naive number of dergees of freedom in the system (i.e., before constraints).\n\n\n\n\n\n","category":"method"},{"location":"reference/#SimpleStatics.original_array-Tuple{AbstractArray}","page":"Index","title":"SimpleStatics.original_array","text":"Recover the original array where DOF indices will match with the StaticSetup.\n\n\n\n\n\n","category":"method"},{"location":"reference/#SimpleStatics.set_force!-Union{Tuple{T}, Tuple{StaticSetup{T}, Int64, SimpleStatics.Vector2D{T}}} where T","page":"Index","title":"SimpleStatics.set_force!","text":"Set the force at a joint, in Newtons.\n\n\n\n\n\n","category":"method"},{"location":"reference/#SimpleStatics.solve_member_stresses-Union{Tuple{T}, Tuple{StaticSetup{T}, Vector{<:SimpleStatics.Vector2D}}} where T","page":"Index","title":"SimpleStatics.solve_member_stresses","text":"Negative -> Compressive, Positive -> Tensile, index matches member index\n\n\n\n\n\n","category":"method"},{"location":"reference/#SimpleStatics.terminal_joints-Tuple{StaticSetup, Integer}","page":"Index","title":"SimpleStatics.terminal_joints","text":"Get the joints IDs that a member connects, found by its member ID.\n\n\n\n\n\n","category":"method"},{"location":"","page":"Home","title":"Home","text":"CurrentModule = SimpleStatics","category":"page"},{"location":"#SimpleStatics","page":"Home","title":"SimpleStatics","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"Documentation for SimpleStatics.","category":"page"},{"location":"#What-is-SimpleStatics?","page":"Home","title":"What is SimpleStatics?","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"SimpleStatics is a package that assists in the computation and visualization of 2D structure stresses and strains. It's geared towards being able to quickly set up, view, and solve statics problems (in that order) while being as easy as possible to learn how to use. ","category":"page"},{"location":"#Who-is-this-package-designed-for?","page":"Home","title":"Who is this package designed for?","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"The target audience are hobbyists wanting to design and build structures for various purposes, such as greenhouses, a wing for an RC plane, you get the idea. Many concepts in this package are directly implemented from the very good book Matrix Analysis of Structures by Kassimali.","category":"page"},{"location":"#How-do-I-get-started?","page":"Home","title":"How do I get started?","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"Check out the Example Workflow","category":"page"},{"location":"#Where-can-I-find-more-examples?","page":"Home","title":"Where can I find more examples?","text":"","category":"section"},{"location":"#[Example-1-Mild-Steel-Truss](@ref)","page":"Home","title":"Example 1 - Mild Steel Truss","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"A simple truss made from mild steel, where we get to see some significant deformations due to loading.","category":"page"}]
}
