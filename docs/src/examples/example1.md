# Example 1 - Mild Steel Warren Truss


In this example, we're going to play around with a few design parameters for a steel truss, where we want to see what size of materials we can get away with in certain sections of the truss in order to minimize the cost. 

The first thing we need to do is make a function that will generate a truss for us, given a few parameters. Here, we'll focus on making the function `build_truss`, where the number of cross member sections (*n*), the width, height, and top load will be considered. 




First, we need to set up the geometry of the system. 
```@example d1
using SimpleStatics

width = (23 * 12 + 10) * 0.0254 # 23', 10" long -> Meters
height = 18 * 0.0254 # 18" tall -> Meters
```

I'll introduce this function all at once, but don't worry, we'll go over every part in detail!

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

So that was a *lot* of code. Before we pick this apart, lets just take a look at with with, say, n=3 to make sure that it works.

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

Then, we start initialize our setup and define `segment_length`, or the length of 

```Julia
s = StaticSetup()
    
segment_len = width_m / (2*n)

top_joint_indices = []
for m in range(0, width_m, 2n + 1)
    j = add_joint!(s, m, height_m)
    push!(top_joint_indices, j)
end
```