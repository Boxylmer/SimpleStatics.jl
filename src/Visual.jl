BACKGROUND_COLOR = "antiquewhite"
LINE_THICKNESS = 2
FONT_SIZE = 16
POINT_SIZE = 4

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

function redistribute_linearly_alpha(values, desired_average)
    non_zero_vals = filter(x -> !(x ≈ 0), values)
    average = sum(non_zero_vals) / length(non_zero_vals)
    return desired_average / average
end

function redistribute_linearly(values, alpha)
    T = [alpha * v for v in values]
    return T
end


function draw_label(pt, text, color="black", offset = Point(8, -8))
    defaults()
    halign = offset.x < 0 ? :right : :left
    valign = offset.y > 0 ? :top : :bottom
    textpath(text, pt + offset, :path, valign=valign, halign=halign)

    sethue(color); fillpreserve()
    setline(0.25)
    sethue(BACKGROUND_COLOR); strokepath()
end

function defaults()
    setline(LINE_THICKNESS)
    fontsize(FONT_SIZE)
    setdash("solid")
end

"""
    plot_setup(setup, name="default"; [dsize=800, padding, displacements, stresses, reactions])

# Arguments
- `setup::StaticSetup`: The setup to plot. 
- `name::String`: The filename or path (without extension) of the image to be created.
- `dsize::Integer`: The diagonal pixel size of the image. The actual width and height will depend on the setup being plotted. 
- `padding::Float64`: The amount of extra distance to include on the border of the setup boundaries. 0 padding will make the image as small as possible, and you will likely lose details as they go off of the screen. Default is 0.4.

# Additional details that can be included.
- `displacements`: Previously calculated displacements using `solve_displacements`.
- `stresses`: Previously calculated member stresses using `solve_member_stresses`.
- `reactions`: Previously calculated reaction forces using `solve_reaction_forces`.
"""
function plot_setup(s::StaticSetup, name="default"; dsize=800, padding=0.4, displacements=nothing, stresses=nothing, reactions=nothing)
    
    xdomain, ydomain = find_domain(s.positions, padding)
    xlen = xdomain[2] - xdomain[1]
    ylen = ydomain[2] - ydomain[1]
    
    mag = sqrt(xlen^2 + ylen^2)
    
    xunit = xlen / mag
    yunit = ylen / mag
    
    size = (xunit * dsize, yunit * dsize)
    pts = transform_for_luxor(s.positions, size, xdomain, ydomain)

    # @svg begin
    svgim = Drawing(size[1], size[2], :svg, name * ".svg")
    defaults()
    origin()
    background(BACKGROUND_COLOR)

    
    sethue("black")
    ### Members
    member_strings = Vector{String}(undef, length(member_ids(s)))
    member_points = Vector{Point}(undef, length(member_ids(s)))
    if !isnothing(displacements); setdash("longdashed"); end
    for mid in member_ids(s)
        i, j = s.edge_to_vertices[mid]
        p1 = pts[i]
        p2 = pts[j]
        line(p1, p2, action = :stroke)
        # draw_label("M"*string(mid), midpoint(p1, p2), color)
        member_strings[mid] = "M"*string(mid)
        member_points[mid] = midpoint(p1, p2)
        # label("M"*string(mid), :NE, midpoint(p1, p2))
    end
    setdash("solid")
    
    ### Displacement Joints
    if !isnothing(displacements)
        sethue("green")
        displaced_pts = transform_for_luxor(s.positions .+ displacements, size, xdomain, ydomain) 

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

        sethue("lime")
        for (i, p) in enumerate(pts)
            circle(displaced_pts[i], POINT_SIZE, action = :fill)
        end
    end
    

    ### Forces
    force_points = Vector{Point}()
    force_strings = Vector{String}()

    average_relative_force_length = 0.10 * sqrt(size[1]^2 + size[2]^2) # average force should consume 10% of the diagonal window size

    sethue("mediumvioletred")
    nonzero_force_indices = [i for i in eachindex(s.forces) if (s.forces[i].x != 0 || s.forces[i].y != 0)]
    nonzero_force_vectors = [s.forces[i] for i in nonzero_force_indices]
    nonzero_force_magnitudes = norm.(nonzero_force_vectors)
    nonzero_force_unit_vectors = [Point(v.x, -v.y) for v in unit_vector.(nonzero_force_vectors)]

    force_to_pixel_alpha = redistribute_linearly_alpha(nonzero_force_magnitudes, average_relative_force_length)
    nonzero_force_pixel_lengths = redistribute_linearly(nonzero_force_magnitudes, force_to_pixel_alpha)
    nonzero_force_pixel_vectors = nonzero_force_pixel_lengths .* nonzero_force_unit_vectors
    for (i, j) in enumerate(nonzero_force_indices)
        p1 = pts[j]
        p2 = pts[j] + nonzero_force_pixel_vectors[i]
        arrow(p1, p2, linewidth=4, arrowheadlength=20, arrowheadangle=pi/6)
        # label("F"*string(i), :SW, midpoint(p1, p2))
        push!(force_points, midpoint(p1, p2))
        push!(force_strings, "F"*string(i))
    end

    ### Reaction Forces
    reaction_points = Vector{Point}()
    reaction_strings = Vector{String}()
    sethue("royalblue1")
    if !isnothing(reactions) && length(nonzero_force_indices) > 0
        nonzero_reaction_indices = [i for i in eachindex(reactions) if !(s.constraints[i] isa NoConstraint)]
        nonzero_reaction_vectors = [reactions[i] for i in nonzero_reaction_indices]
        nonzero_reaction_magnitudes = norm.(nonzero_reaction_vectors)
        nonzero_reaction_unit_vectors = [Point(v.x, -v.y) for v in unit_vector.(nonzero_reaction_vectors)]
        nonzero_reaction_pixel_lengths = redistribute_linearly(nonzero_reaction_magnitudes, force_to_pixel_alpha) # reuse previous alpha to keep the forces "to scale" with each other
        nonzero_reaction_pixel_vectors = nonzero_reaction_pixel_lengths .* nonzero_reaction_unit_vectors

        for (i, j) in enumerate(nonzero_reaction_indices)
            p1 = pts[j]
            p2 = pts[j] + nonzero_reaction_pixel_vectors[i]
            arrow(p1, p2, linewidth=4, arrowheadlength=20, arrowheadangle=pi/6)
            push!(reaction_strings, "RF"*string(i))
            push!(reaction_points, midpoint(p1, p2))
        end
    end

    ### constraints
    sethue("green")
    rectminor = 8
    rectmajor = 32
    for i in eachindex(s.constraints)
        p = pts[i]
        c = s.constraints[i]
        if c isa AnchorConstraint
            circle(p, 6, action = :stroke)
        elseif c isa XRollerConstraint
            pt1 = Point(p[1] - rectmajor/2, p[2] - rectminor/2)
            pt2 = Point(p[1] + rectmajor/2, p[2] - rectminor/2)
            line(pt1, pt2; action=:stroke)

            pt1 = Point(p[1] - rectmajor/2, p[2] + rectminor/2)
            pt2 = Point(p[1] + rectmajor/2, p[2] + rectminor/2)
            line(pt1, pt2; action=:stroke)
        elseif c isa YRollerConstraint
            pt1 = Point(p[1] - rectminor/2, p[2] - rectmajor/2)
            pt2 = Point(p[1] - rectminor/2, p[2] + rectmajor/2)
            line(pt1, pt2; action=:stroke)

            pt1 = Point(p[1] + rectminor/2, p[2] - rectmajor/2)
            pt2 = Point(p[1] + rectminor/2, p[2] + rectmajor/2)
            line(pt1, pt2; action=:stroke)
        else
            nothing
        end
        
    end


    ### Joints
    joint_points = Vector{Point}(undef, length(pts))
    joint_strings = Vector{String}(undef, length(pts))
    for (i, p) in enumerate(pts)
        sethue("black")
        circle(p, POINT_SIZE, action = :fill)

        joint_points[i] = p
        joint_strings[i] = "J"*string(i)
    end



    ### Labels!
    for i in eachindex(joint_points, joint_strings)
        draw_label(joint_points[i], joint_strings[i], "black")
    end
    for i in eachindex(member_points, member_strings)
        draw_label(member_points[i], member_strings[i], "black")
    end
    for i in eachindex(reaction_points, reaction_strings)
        draw_label(reaction_points[i], reaction_strings[i], "royalblue1", Point(-8, 8))
    end
    for i in eachindex(force_points, force_strings)
        draw_label(force_points[i], force_strings[i], "mediumvioletred", Point(-8, 8))
    end
    finish()
    return svgim
end
