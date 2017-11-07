# adapted from plots because its not in Recipes Base
function partialcircle(start_θ, end_θ, n = 20, r=1)
    [r*cos(u) for u in linspace(start_θ, end_θ, n)],
    [r*sin(u) for u in linspace(start_θ, end_θ, n)]
end

function make_circle(cx, cy, r)
    x, y = partialcircle(0, 2pi, 20, r)
    x .+= cx
    y .+= cy
    collect(zip(x, y))
end

@recipe function plot(env::CartpoleEnvironment; force=0.)
    sim = env.sim
    xlims --> (sim.xlimits[1], sim.xlimits[2])
    ylims --> sim.pole_length .* (-1.2,1.2)
    ratio --> true
    xlabel --> "x [meter]"
    ylabel --> "y [meter]"
    title --> @sprintf("Cartpole at t=%0.2f s", sim.time)
    min_force, max_force = extrema(env.actions)
    cart_pos = configuration(sim.state)[1]
    cart_offset = -0.04
    cart_height = 0.1
    cart_width = 0.6
    arrow_height = cart_height / 2
    arrow_width_plus = 0.15 / max_force * force
    arrow_width_minus = 0.15 / min_force * force
    pole_width = sim.pole_com == sim.pole_length ? 0.005 : 0.02
    wheel_radius = 0.07
    wheel_offset = 0.2
    wheel_turns = (cart_pos+10) / (2pi * wheel_radius)
    wheel_angle = -2pi*(wheel_turns - floor(wheel_turns))
    actual_xlims = plotattributes[:xlims]
    @series begin # floor
        seriescolor := :grey
        linewidth := 1
        label := ""
        [actual_xlims...], cart_offset .- cart_height .- wheel_radius .+ zeros(2)
    end
    if force > 0
        @series begin
            seriescolor := :red
            seriestype := :shape
            alpha := 0.2
            label := ""
            (cart_pos .+ cart_width./2 .+ [0, 0, arrow_width_plus, arrow_width_plus*1.2, arrow_width_plus],
             cart_offset .- cart_height./2 .+ arrow_height./2 .* [-1, 1, 1, 0, -1])
        end
    elseif force < 0
        @series begin
            seriescolor := :red
            seriestype := :shape
            alpha := 0.2
            label := ""
            (cart_pos .- cart_width./2 .- [0, 0, arrow_width_minus, arrow_width_minus*1.2, arrow_width_minus],
             cart_offset .- cart_height./2 .+ arrow_height./2 .* [-1, 1, 1, 0, -1])
        end
    end
    @series begin # triangle
        seriescolor := :lightgrey
        seriestype := :shape
        label := ""
        cart_pos .+ [-.1, 0., .1], cart_offset .+ [0., 0.1, 0.]
    end
    @series begin # big cart
        seriescolor := :lightgrey
        seriestype := :shape
        label := ""
        cart_pos .+ cart_width .* [-1, 1, 1, -1] ./ 2, cart_offset .+ [-cart_height, -cart_height, 0., 0.]
    end
    @series begin # wheel left
        seriestype := :shape
        seriescolor := :grey
        label := ""
        make_circle(cart_pos - wheel_offset, cart_offset - cart_height, wheel_radius)
    end
    @series begin # wheel right
        seriestype := :shape
        seriescolor := :grey
        label := ""
        make_circle(cart_pos + wheel_offset, cart_offset - cart_height, wheel_radius)
    end
    wheel_x = [0.7wheel_radius*cos(wheel_angle)]
    wheel_y = [0.7wheel_radius*sin(wheel_angle)]
    @series begin # wheel left scatter
        seriescolor := :black
        seriestype := :scatter
        markersize := 1
        label := ""
        wheel_x .+ cart_pos .- wheel_offset, wheel_y .+ cart_offset .- cart_height
    end
    @series begin # wheel right scatter
        seriescolor := :black
        seriestype := :scatter
        markersize := 1
        label := ""
        wheel_x .+ cart_pos .+ wheel_offset, wheel_y .+ cart_offset .- cart_height
    end
    p_bot_l = transform(sim.state,
            Point3D(frame_after(joints(sim.mechansim)[3]), SVector(-pole_width,0,0.)),
            default_frame(bodies(sim.mechansim)[1])).v
    p_bot_r = transform(sim.state,
            Point3D(frame_after(joints(sim.mechansim)[3]), SVector(pole_width,0,0.)),
            default_frame(bodies(sim.mechansim)[1])).v
    p_top_l = transform(sim.state,
            Point3D(frame_after(joints(sim.mechansim)[3]), SVector{3}(-pole_width,0,sim.pole_length)),
            default_frame(bodies(sim.mechansim)[1])).v
    p_top_r = transform(sim.state,
            Point3D(frame_after(joints(sim.mechansim)[3]), SVector{3}(pole_width,0,sim.pole_length)),
            default_frame(bodies(sim.mechansim)[1])).v
    @series begin # pole itself
        seriescolor := :grey
        seriestype := :shape
        label := ""
        [p_bot_l[1],p_bot_r[1],p_top_r[1],p_top_l[1]], [p_bot_l[3],p_bot_r[3],p_top_r[3],p_top_l[3]]
    end
    if sim.pole_com == sim.pole_length
        p_bot_c = transform(sim.state,
            Point3D(frame_after(joints(sim.mechansim)[3]), SVector(0,0,0.)),
            default_frame(bodies(sim.mechansim)[1])).v
        p_top_c = transform(sim.state,
            Point3D(frame_after(joints(sim.mechansim)[3]), SVector(0.,0,sim.pole_length)),
            default_frame(bodies(sim.mechansim)[1])).v
        @series begin # sphere on top
            seriestype := :shape
            seriescolor := :grey
            label := ""
            make_circle(p_top_c[1], p_top_c[3], 0.07)
        end
    end
    @series begin # sphere on bottom
        seriestype := :shape
        seriescolor := :lightgrey
        label := ""
        make_circle(cart_pos, 0., 0.03)
    end
    if get(plotattributes, :legend, :best) != :none
        @series begin # x line
            seriescolor := :grey
            linewidth := 1
            linestyle := :dash
            label := ""
            [cart_pos, cart_pos], [cart_offset .- cart_height .- wheel_radius .- 0.05, sim.pole_length]
        end
        @series begin # x text
            seriescolor := :grey
            label := ""
            series_annotations := [@sprintf("x = %0.2f", cart_pos), ""]
            [cart_pos], [cart_offset .- cart_height .- wheel_radius .- 0.15]
        end
    end
    nothing
end
