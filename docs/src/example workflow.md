# Example Workflow
As the name might suggest, structures can be created and analyzed with fair ease, with each problem being started with

```@example 1
using SimpleStatics
my_setup = StaticSetup()
```

creating an empty statics problem which can then be filled with members, joints, forces, etc.

Lets create a simple truss to analyze. 


## 1. Add Joints
First, we need to add some **joints**. Joints act as terminals to members, which we will add in the next step. They also need **constraints** to dictate their movement behavior. By default, they are assigned [`NoConstraint()`](@ref). For our first joint, which we want to keep fixed in place, we'll give it an `AnchorConstraint()`.

We can add joints using the [`add_joint!(setup, x, y, constraint)`](@ref) method, which will return the index of the joint that was created. In this case, since we don't have too many joints to add, we can simply assign them to individual variables.

!!! tip "Visualizing your progress"
    You can look at what you've created at any point by calling the [`plot_setup(::StaticSetup)`](@ref) function on your setup! Below we will use this to see what we've created in our example workflow.
    *The use of this function is shown in step 1, but is hidden in future steps.* 


```@example 1
j1 = add_joint!(my_setup, 0, 0, AnchorConstraint())
j2 = add_joint!(my_setup, 1, 0)
j3 = add_joint!(my_setup, 1, 1)
j4 = add_joint!(my_setup, 2, 0)
j5 = add_joint!(my_setup, 2, 1)
j6 = add_joint!(my_setup, 3, 0)
j7 = add_joint!(my_setup, 3, 1)
j8 = add_joint!(my_setup, 4, 0)

plot_setup(my_setup)
```

**Notice the following**
- Joint 1 was given an [`AnchorConstraint()`](@ref) which will keep it fixed in place. 
- Points were automatically labeled in the order they were created. 
- Constraints were depicted in green denoting the kind of motion allowed for the joint. 

If we want to modify constraints after the joint was already added, we can do so using the `set_constraint!` function. Lets add an `XRollerConstraint()` to **J8**, as we want it to stay fixed on the Y axis, but be allowed to move along the X axis.

```@example 1
set_constraint!(my_setup, j8, XRollerConstraint())

plot_setup(my_setup) # hide
```

## 2. Add Members

Now we need to connect our joints via **members**. 

Similar to how a **joint** 's behavior is controlled by its **constraint**, a **member**'s behavior is controlled by its **material**, these are accessed via the `Materials` submodule.
Members have a default for this as well: A `PerfectMaterial()`, which is a very thick material with a tensile strength a few orders higher than that of Tungsten, the strongest rote metal. 

Lets use standard 2x4 lumber for our truss structure.

```@example 1
m = Materials.Pine2x4()
```

We can now add members using the [`add_member!(setup, j1, j2, material)`](@ref) function. This function will also return an identifier for each member that could be used to refer to them later on, but since we don't need to do anything directly to them after this, we don't need to assign them to anything. 

Lets set up a more proper-looking truss. 


```@example 1
add_member!(my_setup, j1, j2, m)
add_member!(my_setup, j1, j3, m)
add_member!(my_setup, j2, j3, m)
add_member!(my_setup, j2, j4, m)
add_member!(my_setup, j3, j4, m)
add_member!(my_setup, j3, j5, m)
add_member!(my_setup, j4, j5, m)
add_member!(my_setup, j4, j6, m)
add_member!(my_setup, j5, j6, m)
add_member!(my_setup, j5, j7, m)
add_member!(my_setup, j6, j7, m)
add_member!(my_setup, j6, j8, m)
add_member!(my_setup, j7, j8, m)
plot_setup(my_setup) # hide
```

Whew! That was a lot of members. This can be tedious, but it's a necessary step in creating our masterpiece. Looking at the result immediately shows us the fruits of our labor, as we now have a very nice looking structure. 


# 3. Add Forces
This truss isn't very useful to us unless we understand more about how it handles loads! 

We can add forces to joints by using the `set_force!(setup, j, fx, fy)` function. Lets add a $50lbf$ ($222.411N$) load directly on the top of the truss, which would be at **J5** in the -Y direction. 

```@example 1
set_force!(my_setup, j5, 0, -222.411)
plot_setup(my_setup) # hide
```

Now we can see a very nice pink arrow depicting our force and how it's acting on our structure!


# 4. Calculate Displacements

After completing step 3, we've fully defined a statics problem. Now the question is: *What can we actually do with it?*
The first question is generally centered around how the forces acting on the truss deforms it, which we can easily calculate using the `solve_displacements(setup)` function!

!!! tip "Joint indices match displacements"
    The displacements we get out of this funciton will follow the same order as the joints we created, so we can see the displacement that happened to **J5**, we simply access it in the calculated displacements using `j5`.
    We can also access each component of the displacements via `displacement[idx].x` or -`.y` if needed. 

```@example 1
displacements = solve_displacements(my_setup)
displacements[j5]
```
It looks like **J5** moved down by about $3.4\times10^{-5}m$ or $0.00137"$. This certainly isn't a lot. Let see what happened visually as well to confirm this for all of our points!

This time our plot_setup function will need to have the `displacements` data as well to be able to show us what actually happened. 

```@example 1
plot_setup(my_setup, displacements=displacements)
```
Members are now displayed in green to show us the displaced structure, and sure enough, none of the points really moved. Lets try adding a few thousand more pounds to **J5** and seeing what happens.

```@example 1
set_force!(my_setup, j5, 0, -22241.11) # about 5000 pounds force. 
displacements = solve_displacements(my_setup)
displacements[j5]
```

```@example 1
plot_setup(my_setup, displacements=displacements) # hide
```

Hmm... we're applying $5k lbf$ and only seeing about $0.4"$ of sag? That doesn't sound right. Surely 2x4 lumber isn't that strong? There's good news and bad news here. 
- Good news: The 2x4 lumber you built your truss with is *absolutely* that strong.
- Bad news: The screws you used to hold them together were *not*. 


# 5. Calculate Member Stresses
Okay, so how do we solve this? 
Let's say a standard deck screw can hold $100lbf$ ($444.822N$)  comfortably before it shears, and we use two screws per board, getting about $850N$ (conservatively) of force that each board (member) can handle before it breaks. 

We can calculate the stress each board is under with the `solve_member_stresses(setup, displacements)` function. 

!!! tip "Remember, plot_setup is your friend!"
    We can also pass the result of `solve_member_stresses` into our `plot_setup` function using the `stresses` keyword! Though these will only be visible if `displacements` are also present. 
    

```@example 1
member_stresses = solve_member_stresses(my_setup, displacements)
maximum(member_stresses), minimum(member_stresses) # in Newtons
```

First and foremost we notice that, as expected, our truss will be destroyed ten times over given the maximum stress on the members. 
!!! note "Sign conventions are important!"
    Conventionally, negative stresses indicate **tension** and positive stresses indicate **compression**. This signage is arbitrary, but it's what was chosen for this library.

Lets switch back to our $50lbf$ setup and check the maximum member force to see if we can at least bear that load without the risk of failure.

```@example 1
set_force!(my_setup, j5, 0, -222.411) # about 50 pounds force. 
displacements = solve_displacements(my_setup)
member_stresses = solve_member_stresses(my_setup, displacements)
```

```@example 1
plot_setup(my_setup, displacements=displacements, stresses=member_stresses) # hide
```

Now we have some interesting results! Lets go over them, noting that compressive forces are shown in red and tensile forces in blue. 
- Notice that **M3** and **M11** are so-called *zero force members*. These members technically don't bear any load, but are necessary to constrain the system of equations that our setup has generated.
- **M6** and **M8** have equal and opposite stresses equal to the load on **J5**, whereas **M9** and **M10**, as well as **M4** and **M5** are much lower, due to the load being distributed (unevenly) between the two. 


What will happen if we modify our truss slightly by recreating it with a member from **J4** to **J7** rather than **J5** to **J6**. 

```@example 1
    my_setup = StaticSetup()
    j1 = add_joint!(my_setup, 0, 0, AnchorConstraint())
    # ...
    j2 = add_joint!(my_setup, 1, 0) # hide
    j3 = add_joint!(my_setup, 1, 1) # hide
    j4 = add_joint!(my_setup, 2, 0) # hide
    j5 = add_joint!(my_setup, 2, 1) # hide
    j6 = add_joint!(my_setup, 3, 0) # hide
    j7 = add_joint!(my_setup, 3, 1) # hide
    j8 = add_joint!(my_setup, 4, 0, XRollerConstraint()) # hide
    add_member!(my_setup, j1, j2, m) # hide
    add_member!(my_setup, j1, j3, m) # hide
    add_member!(my_setup, j2, j3, m) # hide
    add_member!(my_setup, j2, j4, m) # hide
    add_member!(my_setup, j3, j4, m) # hide
    add_member!(my_setup, j3, j5, m) # hide
    add_member!(my_setup, j4, j5, m) # hide
    add_member!(my_setup, j4, j6, m) # hide
    add_member!(my_setup, j4, j7, m) # hide
    add_member!(my_setup, j5, j7, m) # hide
    add_member!(my_setup, j6, j7, m) # hide
    add_member!(my_setup, j6, j8, m) # hide
    add_member!(my_setup, j7, j8, m) # hide
    set_force!(my_setup, j5, 0, -222.411)
    displacements = solve_displacements(my_setup)
    member_stresses = solve_member_stresses(my_setup, displacements)
```

```@example 1
plot_setup(my_setup, displacements=displacements, stresses=member_stresses) # hide
```

Apparently nothing good, as now one extra member is under a significantly higher load. What if we add a few extra cross members? 
```@example 1
    add_member!(my_setup, j2, j5, m) 
    add_member!(my_setup, j5, j6, m) 
    displacements = solve_displacements(my_setup)
    member_stresses = solve_member_stresses(my_setup, displacements)
    maximum(member_stresses), minimum(member_stresses)
```

```@example 1
plot_setup(my_setup, displacements=displacements, stresses=member_stresses) # hide
```

The result is a bit messy, but it looks like the maximum member stress was reduced!


# 6. Calculate Reaction Forces

We can see how these forces are affecting our constraints by using the `solve_reaction_forces(setup, displacements)` function, then passing in the result to our `plot_setup` function with the `reactions` keyword.

```@example 1
    reaction_forces = solve_reaction_forces(my_setup, displacements)
    plot_setup(my_setup, displacements=displacements, stresses=member_stresses, reactions=reaction_forces)
```