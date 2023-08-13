# Example 1 - Mild Steel Warren Truss


In this example, we're going to play around with a few design parameters for a Warren-like steel truss with verticals, where we want to see what size of materials we can get away with in certain sections of the truss in order to minimize the cost. 

We will do this by:
1. [Making a function that will generate a truss based on a few parameters.](@ref d1_part1)
2. [Generating a number of trusses with a certain load and analyzing it.](@ref d1_part2)
3. [Viewing stresses and finding structural weak points.](@ref d1_part3)


## [1. Making the setup function](@id d1_part1)

The first thing we need to do is make a function that will generate a truss for us, given a few parameters. Here, we'll focus on making the function `build_truss`, where the number of cross member sections (*n*), the width, height, and top load will be considered. 

We will start by making the geometry of the system. 


```@example d1
using SimpleStatics

width = (23 * 12 + 10) * 0.0254 # 23', 10" long -> Meters
height = 18 * 0.0254 # 18" tall -> Meters
nothing # hide
```

I'll introduce this function all at once, but we'll go over every part in detail after that!

```@example d1
function build_truss(width_m, height_m, n, top_load = 0; 
    thick_material=Materials.SquareTubing(Materials.MildSteel, 1.5 * 0.0381, 1.897 / 1000),   # 1.5" steel tubing with 14 gauge thickness
    medium_material=Materials.SquareTubing(Materials.MildSteel, 1.0 * 0.0381, 1.518 / 1000),  # 1.0" steel tubing with 16 gauge thickness
    thin_material=Materials.SquareTubing(Materials.MildSteel, 0.75 * 0.0381, 1.518 / 1000)    # .75" steel tubing with 16 gauge thickness
    )

    s = StaticSetup()
    
    m = width_m / (2*n)
    
    top_joint_indices = []
    for m in range(0, width_m, 2n + 1)
        j = add_joint!(s, m, height_m)
        push!(top_joint_indices, j)
    end
    
    bottom_joint_indices = []
    for m in range(m, width_m - m, n)
        j = add_joint!(s, m, 0)
        push!(bottom_joint_indices, j)
    end
    
    top_bar_members = []
    for i in top_joint_indices[1:end-1]
        m = add_member!(s, i, i+1)
        push!(top_bar_members, m, thick_material)
    end

    bottom_bar_members = []
    for i in bottom_joint_indices[1:end-1]
        m = add_member!(s, i, i+1)
        push!(bottom_bar_members, m, medium_material)
    end

    cross_members = []
    for i in 1:n
        for j in -1:1
            m = add_member!(s, bottom_joint_indices[i], top_joint_indices[2*i + j], thin_material)
            push!(cross_members, m)
        end
    end
    set_constraint!(s, top_joint_indices[1], AnchorConstraint())
    set_constraint!(s, top_joint_indices[end], XRollerConstraint())
    
    SimpleStatics.add_member_weights!(s)
    # add_force!(s, top_joint_indices[n+1], 0, -top_load) # middle of top bar
    for i in top_joint_indices
        add_force!(s, i, 0, -top_load/length(top_joint_indices)) 
    end
    return s
end
```

So that was a bit of code to chew on all at once. Before we pick this apart, lets just take a look at what this function creates with n=3 to make sure that it works.

```@example d1
plot_setup(build_truss(width, height, 3), dsize=1500)
```

So, in this function we start by defining some materials and dimensions, as well as the number of cross member repeat units `n` and the `top_load` that will be distributed over our truss itself. 
```Julia
function build_truss(width_m, height_m, n, top_load = 0; 
    thick_material=Materials.SquareTubing(Materials.MildSteel, 1.5 * 0.0381, 1.897 / 1000),   # 1.5" steel tubing with 14 gauge thickness
    medium_material=Materials.SquareTubing(Materials.MildSteel, 1.0 * 0.0381, 1.518 / 1000),  # 1.0" steel tubing with 16 gauge thickness
    thin_material=Materials.SquareTubing(Materials.MildSteel, 0.75 * 0.0381, 1.518 / 1000)    # .75" steel tubing with 16 gauge thickness
    )
```

Then, we start initialize our setup and define `segment_length`, or the length of each member of the top bar in the truss. A truss with `n` repeats will have `2n+1` joints in its top bar.

```Julia
s = StaticSetup()
    
segment_len = width_m / (2*n)

top_joint_indices = []
for m in range(0, width_m, 2n + 1)
    j = add_joint!(s, m, height_m)
    push!(top_joint_indices, j)
end
```

And `n` joints at the bottom bar, which is two `segment_lengths` shorter than the top bar, so we have to shift the starting joint in by a `segment_length` and stop one `segment_length` early. 

```Julia
bottom_joint_indices = []
for m in range(segment_len, width_m - segment_len, n)
    j = add_joint!(s, m, 0)
    push!(bottom_joint_indices, j)
end
```

We can then connect all the top and bottom truss members, recording their indexes while doing so.
```Julia
top_bar_members = []
for i in top_joint_indices[1:end-1]
    m = add_member!(s, i, i+1)
    push!(top_bar_members, m, thick_material)
end

bottom_bar_members = []
for i in bottom_joint_indices[1:end-1]
    m = add_member!(s, i, i+1)
    push!(bottom_bar_members, m, medium_material)
end
```

We can visualize what we've created so far with our [`plot_setup(setup)`](@ref) function


```@setup d1
function build_truss(width_m, height_m, n, top_load = 0; 
    thick_material=Materials.SquareTubing(Materials.MildSteel, 1.5 * 0.0381, 1.897 / 1000),   # 1.5" steel tubing with 14 gauge thickness
    medium_material=Materials.SquareTubing(Materials.MildSteel, 1.0 * 0.0381, 1.518 / 1000),  # 1.0" steel tubing with 16 gauge thickness
    thin_material=Materials.SquareTubing(Materials.MildSteel, 0.75 * 0.0381, 1.518 / 1000)    # .75" steel tubing with 16 gauge thickness
    )

    s = StaticSetup()
    
    segment_len = width_m / (2*n)
    
    top_joint_indices = []
    for m in range(0, width_m, 2n + 1)
        j = add_joint!(s, m, height_m)
        push!(top_joint_indices, j)
    end
    
    bottom_joint_indices = []
    for m in range(segment_len, width_m - segment_len, n)
        j = add_joint!(s, m, 0)
        push!(bottom_joint_indices, j)
    end
    
    top_bar_members = []
    for i in top_joint_indices[1:end-1]
        m = add_member!(s, i, i+1)
        push!(top_bar_members, m, thick_material)
    end

    bottom_bar_members = []
    for i in bottom_joint_indices[1:end-1]
        m = add_member!(s, i, i+1)
        push!(bottom_bar_members, m, medium_material)
    end
    return s
end
```


```@example d1
truss = build_truss(width, height, 3)
plot_setup(truss)
```

Looks good so far! For the cross members, we want each bottom member to connect to its respective closest three top members.


```Julia
cross_members = []
for i in 1:n
    for j in -1:1
        m = add_member!(s, bottom_joint_indices[i], top_joint_indices[2*i + j], thin_material)
        push!(cross_members, m)
    end
end
```

Lets see how this looks now. 

```@setup d1
function build_truss(width_m, height_m, n, top_load = 0; 
    thick_material=Materials.SquareTubing(Materials.MildSteel, 1.5 * 0.0381, 1.897 / 1000),   # 1.5" steel tubing with 14 gauge thickness
    medium_material=Materials.SquareTubing(Materials.MildSteel, 1.0 * 0.0381, 1.518 / 1000),  # 1.0" steel tubing with 16 gauge thickness
    thin_material=Materials.SquareTubing(Materials.MildSteel, 0.75 * 0.0381, 1.518 / 1000)    # .75" steel tubing with 16 gauge thickness
    )

    s = StaticSetup()
    
    segment_len = width_m / (2*n)
    
    top_joint_indices = []
    for m in range(0, width_m, 2n + 1)
        j = add_joint!(s, m, height_m)
        push!(top_joint_indices, j)
    end
    
    bottom_joint_indices = []
    for m in range(segment_len, width_m - segment_len, n)
        j = add_joint!(s, m, 0)
        push!(bottom_joint_indices, j)
    end
    
    top_bar_members = []
    for i in top_joint_indices[1:end-1]
        m = add_member!(s, i, i+1)
        push!(top_bar_members, m, thick_material)
    end

    bottom_bar_members = []
    for i in bottom_joint_indices[1:end-1]
        m = add_member!(s, i, i+1)
        push!(bottom_bar_members, m, medium_material)
    end

    cross_members = []
    for i in 1:n
        for j in -1:1
            m = add_member!(s, bottom_joint_indices[i], top_joint_indices[2*i + j], thin_material)
            push!(cross_members, m)
        end
    end

    return s
end
```

```@example d1
truss = build_truss(width, height, 3)
plot_setup(truss)
```

Looks good! Finally, we want to add constraints to the first and last top member, so that the truss is held essentially by a fixed point and is able to freely expand and contract horizontally under load. 

```Julia
set_constraint!(s, top_joint_indices[1], AnchorConstraint())
set_constraint!(s, top_joint_indices[end], XRollerConstraint())
```

Finally, we want to add loads to this structure, both from the `top_load` as well as the weights that the members themselves exert on it. Thankfully, we have the convenience function [`add_member_weights!(setup)`](@ref) to do this for us. 

```Julia
SimpleStatics.add_member_weights!(s)
for i in top_joint_indices
    add_force!(s, i, 0, -top_load/length(top_joint_indices)) 
end
```

We can visualize this final setup function and see that the forces and member weights were all applied. 

```@setup d1
function build_truss(width_m, height_m, n, top_load = 0; 
    thick_material=Materials.SquareTubing(Materials.MildSteel, 1.5 * 0.0381, 1.897 / 1000),   # 1.5" steel tubing with 14 gauge thickness
    medium_material=Materials.SquareTubing(Materials.MildSteel, 1.0 * 0.0381, 1.518 / 1000),  # 1.0" steel tubing with 16 gauge thickness
    thin_material=Materials.SquareTubing(Materials.MildSteel, 0.75 * 0.0381, 1.518 / 1000)    # .75" steel tubing with 16 gauge thickness
    )

    s = StaticSetup()
    
    segment_len = width_m / (2*n)
    
    top_joint_indices = []
    for m in range(0, width_m, 2n + 1)
        j = add_joint!(s, m, height_m)
        push!(top_joint_indices, j)
    end
    
    bottom_joint_indices = []
    for m in range(segment_len, width_m - segment_len, n)
        j = add_joint!(s, m, 0)
        push!(bottom_joint_indices, j)
    end
    
    top_bar_members = []
    for i in top_joint_indices[1:end-1]
        m = add_member!(s, i, i+1)
        push!(top_bar_members, m, thick_material)
    end

    bottom_bar_members = []
    for i in bottom_joint_indices[1:end-1]
        m = add_member!(s, i, i+1)
        push!(bottom_bar_members, m, medium_material)
    end

    cross_members = []
    for i in 1:n
        for j in -1:1
            m = add_member!(s, bottom_joint_indices[i], top_joint_indices[2*i + j], thin_material)
            push!(cross_members, m)
        end
    end
    set_constraint!(s, top_joint_indices[1], AnchorConstraint())
    set_constraint!(s, top_joint_indices[end], XRollerConstraint())
    
    SimpleStatics.add_member_weights!(s)
    # add_force!(s, top_joint_indices[n+1], 0, -top_load) # middle of top bar
    for i in top_joint_indices
        add_force!(s, i, 0, -top_load/length(top_joint_indices)) 
    end
    return s
end
```

```@example d1
truss = build_truss(width, height, 3)
plot_setup(truss)
```

Looks like we're ready to move on!


## [2. Generating some trusses and analyzing their properties.](@id d1_part2)

Now we can generate a few structures with different values of `n`, seeing how it affects the weight and strength of our structure. 
Lets try n values from 1 to 10 and look at 1, 5, and 10 to see what's going on. 


```@example d1
width = (23 * 12 + 10) * 0.0254 # 23', 10" long -> Meters
height = 18 * 0.0254 # 18" tall -> Meters
load = 500 * 0.453592 * 9.81 # lbf -> N, 500 lb load 
nvals = 1:10
trusses = [build_truss(width, height, nval, load) for nval in nvals]

plot_setup(trusses[1], dsize=2400)
plot_setup(trusses[5], dsize=2400)
plot_setup(trusses[10], dsize=2400)
plot_setup(trusses[1], dsize=2400) # hide
```
```@example d1
plot_setup(trusses[5], dsize=2400) # hide
```
```@example d1
plot_setup(trusses[10], dsize=2400) # hide
```


This is in line with what we expected. Lets look at how much each truss weighs. 
```@example d1
mass.(trusses) .* 2.20462  # kg -> lbs
```

And finally we can solve the entire system and visualize the results for a couple structures. This time we'll hide the labels as well to be able to see more clearly. 
```@example d1
displacements = solve_displacements.(trusses)
forces = solve_member_forces.(trusses, displacements)
reactions = solve_reaction_forces.(trusses, displacements)
member_stresses = solve_member_stresses.(trusses, forces)


plot_setup(trusses[1]; displacements=displacements[1], member_forces=forces[1], reactions=reactions[1], draw_labels=false)
plot_setup(trusses[5]; displacements=displacements[5], member_forces=forces[5], reactions=reactions[5], draw_labels=false)
plot_setup(trusses[10]; displacements=displacements[10], member_forces=forces[10], reactions=reactions[10], draw_labels=false)
plot_setup(trusses[1]; displacements=displacements[1], member_forces=forces[1], reactions=reactions[1], draw_labels=false) # hide
```
```@example d1
plot_setup(trusses[5]; displacements=displacements[5], member_forces=forces[5], reactions=reactions[5], draw_labels=false) # hide
```
```@example d1
plot_setup(trusses[10]; displacements=displacements[10], member_forces=forces[10], reactions=reactions[10], draw_labels=false) # hide
```

The reaction forces quickly become too large to view on screen, but this is simply because the additional cross members make the structure heavier.


## [3. Viewing stresses rather than forces, and identifying weak members.](@id d1_part3)

Interestingly, we can also view the maximum stress in the structure to see which members are bottlenecking our truss strength. 

!!! tip "Stresses, not forces, are usually what you want to see."
    The default member property to view in the `plot_setup` function is *force*, however, it is often more useful to view *stress*, which is a better indicator of how close a member is to reaching its yield point, or the stress at which permanent deformation (and therefore damage) to the member begins to occur.
    
This is useful as some members may be under comparitively little force but are very close to their yield point as they are thin, or you may want to identify members that are too thick and can be replaced with thinner materials you have on hand without jeopardizing the strength of the structure. 

```@example d1
max_stresses = maximum.(member_stresses)
```

One interesting thing to note in our structure is that we distributed the load *perfectly* over all the top joints, meaning we accidentally created an edge case where some portion of the load, specifcally $(2/2n+1)$ of our load, was distributed to *constrained joints*, which will not affect the system at all. As a consequence, the first three or so trusses actually appear to experience *less* stress than the trusses with more members. For now, we can ignore this, and look further to looking at *stress utilization* rather than *stress* outright. 


```@example d1
max_stresses = maximum.(solve_stress_utilization.(trusses, member_stresses))
```

It looks like this was a non issue for our larger trusses, as we never really end up using more than about 3.5% of our structures strength. 

To visualize this, lets hand `member_forces` our `member_stresses` rather than the `forces` themselves. 
```@example d1
plot_setup(trusses[1]; dsize=1000, displacements=displacements[1], member_forces=member_stresses[1], reactions=reactions[1], draw_labels=false)
```
```@example d1
plot_setup(trusses[5]; dsize=1000, displacements=displacements[5], member_forces=member_stresses[5], reactions=reactions[5], draw_labels=false)
```
```@example d1
plot_setup(trusses[10]; dsize=1000, displacements=displacements[10], member_forces=member_stresses[10], reactions=reactions[10], draw_labels=false)
```

Interesting! So the cross members are unders the most stress. This makes sense, as they're made from the thinnest material we had. Note that since stress is the force over the materials cross sectional area, the member highlighting would be the same for either property if all members were the made from the same material. 