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
    origin()
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
        sethue("green")
        for i in eachindex(s.constraints)
            p = pts[i]
            c = s.constraints[i]
            if c isa AnchorConstraint
                circle(p, 4, action = :stroke)
            elseif c isa XRollerConstraint
                rect(p[1] - 16, p[2] - 2, 32, 4, action = :stroke)
            elseif c isa YRollerConstraint
                rect(p[1] - 2, p[2] - 16, 4, 32, action = :stroke)
            end
        end

    # end size[1] size[2]
    finish()
    preview()
end
