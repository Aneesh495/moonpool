-- Moonpool Subsea Robotics Acoustic Navigation
-- Acoustic Multipath Discrimination, Surface/Seabed Reflection Modeling, and Innovation Gating
-- OpenEuphoria v4.1+ Specification

namespace moonpool_multipath

include std/math.e
include src/core/constants.e
include src/core/math_util.e

-- Calculate theoretical travel times for direct path, surface bounce, and seabed bounce
-- water_depth_m: total seabed depth from surface (meters)
-- z_src: source depth (meters)
-- z_rcv: receiver depth (meters)
-- horiz_range_m: horizontal distance between source and receiver (meters)
-- sound_speed: average water column sound speed (m/s)
-- Returns sequence {t_direct_sec, t_surface_sec, t_seabed_sec}
public function multipath_arrival_times(
    atom water_depth_m,
    atom z_src,
    atom z_rcv,
    atom horiz_range_m,
    atom sound_speed
)
    atom d_direct, d_surface, d_seabed
    atom t_direct, t_surface, t_seabed
    atom r2 = horiz_range_m * horiz_range_m

    -- Direct path
    d_direct = sqrt(r2 + (z_rcv - z_src) * (z_rcv - z_src))
    t_direct = d_direct / sound_speed

    -- Surface bounce (reflected at z = 0)
    d_surface = sqrt(r2 + (z_src + z_rcv) * (z_src + z_rcv))
    t_surface = d_surface / sound_speed

    -- Seabed bounce (reflected at z = water_depth_m)
    atom dz_bottom = (water_depth_m - z_src) + (water_depth_m - z_rcv)
    d_seabed = sqrt(r2 + dz_bottom * dz_bottom)
    t_seabed = d_seabed / sound_speed

    return {t_direct, t_surface, t_seabed}
end function

-- Test whether an observed travel time is an acoustic multipath reflection
-- Returns sequence {is_multipath, bounce_type}
-- bounce_type: 0 = direct, 1 = surface reflection, 2 = seabed reflection, -1 = unknown anomalous
public function classify_acoustic_arrival(
    atom obs_twtt_sec,
    atom water_depth_m,
    atom z_src,
    atom z_rcv,
    atom horiz_range_m,
    atom sound_speed,
    atom tol_sec
)
    sequence times
    atom twtt_direct, twtt_surface, twtt_seabed

    times = multipath_arrival_times(water_depth_m, z_src, z_rcv, horiz_range_m, sound_speed)
    twtt_direct = 2.0 * times[1]
    twtt_surface = 2.0 * times[2]
    twtt_seabed = 2.0 * times[3]

    if abs(obs_twtt_sec - twtt_direct) <= tol_sec then
        return {0, 0} -- Direct arrival verified
    elsif abs(obs_twtt_sec - twtt_surface) <= tol_sec then
        return {1, 1} -- Surface bounce detected
    elsif abs(obs_twtt_sec - twtt_seabed) <= tol_sec then
        return {1, 2} -- Seabed bounce detected
    else
        return {1, -1} -- Anomalous arrival
    end if
end function

-- 1D Mahalanobis innovation distance gating
-- Returns 1 if measurement passes within gate, 0 if rejected as an outlier
public function innovation_gate_1d(atom meas, atom pred, atom variance, atom gate_threshold)
    atom diff = meas - pred
    if variance < EPSILON then
        variance = EPSILON
    end if
    atom d2 = (diff * diff) / variance
    if d2 <= gate_threshold then
        return 1
    else
        return 0
    end if
end function
