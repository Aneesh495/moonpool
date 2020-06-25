-- Moonpool Subsea Robotics Acoustic Navigation
-- Snell's Law Acoustic Ray Tracing and Eigenray Multipath Solver in Stratified Media
-- OpenEuphoria v4.1+ Specification

namespace moonpool_ray_trace

include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/acoustic/sound_speed.e

-- Calculate Snell's acoustic ray parameter: p = cos(theta) / c
-- theta_rad: grazing angle relative to horizontal (radians)
-- c: local sound speed (m/s)
public function snell_parameter(atom theta_rad, atom c)
    return cos(theta_rad) / c
end function

-- Calculate grazing angle theta at depth where sound speed is c given ray parameter p
-- Returns angle in radians, or -1.0 if turning point reached (p*c > 1.0)
public function grazing_angle(atom p, atom c)
    atom arg = p * c
    if arg >= 1.0 then
        return 0.0 -- Horizontal turning point
    elsif arg <= -1.0 then
        return PI
    else
        return arccos(arg)
    end if
end function

-- Trace an acoustic ray across a single linear-gradient layer
-- Returns sequence {dx, dt, ds, theta_exit, reached_boundary}
public function trace_layer(atom p, atom z_in, atom z_out, atom c_in, atom c_out, atom theta_in)
    atom g, dz, cos_in, sin_in, cos_out, sin_out
    atom dx, dt, ds, c_turn, z_turn
    integer reached_boundary = 1

    dz = z_out - z_in
    if abs(dz) < EPSILON then
        return {0.0, 0.0, 0.0, theta_in, 1}
    end if

    g = (c_out - c_in) / dz

    -- Check for iso-velocity layer (g near zero)
    if abs(g) < 1.0e-6 then
        sin_in = sin(theta_in)
        if abs(sin_in) < EPSILON then
            sin_in = sign(theta_in) * EPSILON
        end if
        dx = abs(dz) / tan(abs(theta_in))
        ds = sqrt(dx * dx + dz * dz)
        dt = ds / c_in
        return {dx, dt, ds, theta_in, 1}
    end if

    cos_in = p * c_in
    if abs(cos_in) > 1.0 then
        cos_in = sign(cos_in)
    end if
    sin_in = sin(theta_in)

    -- Check if ray turns inside this layer before reaching z_out
    c_turn = 1.0 / p
    if (g > 0.0 and c_out > c_turn and c_turn > c_in) or
       (g < 0.0 and c_out < c_turn and c_turn < c_in) then
        -- Ray reaches turning point within layer
        reached_boundary = 0
        z_turn = z_in + (c_turn - c_in) / g
        dz = z_turn - z_in
        sin_out = 0.0
        cos_out = 1.0
    else
        cos_out = p * c_out
        if abs(cos_out) > 1.0 then
            cos_out = sign(cos_out)
        end if
        sin_out = sqrt(max({0.0, 1.0 - cos_out * cos_out}))
        if theta_in < 0.0 then
            sin_out = -sin_out
        end if
    end if

    -- Circular arc integration formulas
    dx = abs((sin_in - sin_out) / (g * p))
    
    -- Travel time integration: dt = (1 / |g|) * ln((c_out/c_in) * (1 + sin_in) / (1 + sin_out))
    atom ratio_c = (c_in + g * dz) / c_in
    atom num_term = 1.0 + abs(sin_in)
    atom den_term = 1.0 + abs(sin_out)
    if den_term < EPSILON then
        den_term = EPSILON
    end if
    dt = (1.0 / abs(g)) * abs(log(ratio_c * (num_term / den_term)))
    ds = sqrt(dx * dx + dz * dz) -- Arc chord approximation

    atom theta_exit = arccos(clamp(p * (c_in + g * dz), -1.0, 1.0))
    if theta_in < 0.0 then
        theta_exit = -theta_exit
    end if

    return {dx, dt, ds, theta_exit, reached_boundary}
end function

-- Trace an acoustic ray from z_source to z_target through full SVP profile:
-- profile = {{z0, c0}, {z1, c1}, ...}
-- Returns sequence {total_time, total_range, total_path_len, trajectory}
-- trajectory is a sequence of {range, depth} sounding points
public function ray_trace_profile(sequence profile, atom z_source, atom z_target, atom theta_deg)
    atom theta_rad, c_source, p, cur_z, cur_x, cur_t, cur_s, cur_theta
    atom z_step = 10.0 -- Depth integration resolution (meters)
    integer n_layers
    sequence trajectory = {}

    theta_rad = theta_deg * DEG2RAD
    c_source = svp_interpolate(profile, z_source)
    p = snell_parameter(theta_rad, c_source)

    cur_z = z_source
    cur_x = 0.0
    cur_t = 0.0
    cur_s = 0.0
    cur_theta = theta_rad

    trajectory = append(trajectory, {cur_x, cur_z})

    atom z_dir = sign(z_target - z_source)
    if z_dir = 0.0 then
        z_dir = 1.0
    end if

    while (z_dir > 0.0 and cur_z < z_target) or (z_dir < 0.0 and cur_z > z_target) do
        atom next_z = cur_z + z_dir * z_step
        if z_dir > 0.0 and next_z > z_target then
            next_z = z_target
        elsif z_dir < 0.0 and next_z < z_target then
            next_z = z_target
        end if

        atom c_in = svp_interpolate(profile, cur_z)
        atom c_out = svp_interpolate(profile, next_z)

        sequence step_res = trace_layer(p, cur_z, next_z, c_in, c_out, cur_theta)
        cur_x += step_res[1]
        cur_t += step_res[2]
        cur_s += step_res[3]
        cur_theta = step_res[4]
        cur_z = next_z

        trajectory = append(trajectory, {cur_x, cur_z})

        if step_res[5] = 0 then
            -- Ray turned
            exit
        end if
    end while

    return {cur_t, cur_x, cur_s, trajectory}
end function

-- Find eigenray takeoff angle theta_deg between source (0, z_src) and receiver (target_r, z_tgt)
-- using bisection search
public function find_eigenray(sequence profile, atom z_src, atom z_tgt, atom target_r, atom tol_m, integer max_iter)
    atom low_deg = -85.0
    atom high_deg = 85.0
    atom mid_deg = 0.0
    sequence res

    for iter = 1 to max_iter do
        mid_deg = (low_deg + high_deg) * 0.5
        res = ray_trace_profile(profile, z_src, z_tgt, mid_deg)
        atom range_diff = res[2] - target_r

        if abs(range_diff) <= tol_m then
            return {mid_deg, res[1], res[2]} -- {takeoff_angle_deg, travel_time_sec, actual_range_m}
        end if

        if range_diff > 0.0 then
            -- Steeper angle needed to shorten horizontal range
            if mid_deg > 0.0 then
                low_deg = mid_deg
            else
                high_deg = mid_deg
            end if
        else
            if mid_deg > 0.0 then
                high_deg = mid_deg
            else
                low_deg = mid_deg
            end if
        end if
    end for

    return {mid_deg, res[1], res[2]}
end function
