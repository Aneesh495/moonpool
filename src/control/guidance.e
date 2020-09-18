-- Moonpool Subsea Robotics Propulsion and Control
-- 3D Line-of-Sight (LOS) Subsea Waypoint Guidance Law
-- OpenEuphoria v4.1+ Specification

namespace moonpool_guidance

include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e

-- 3D Line-of-Sight (LOS) Guidance calculation:
-- veh_pos_ned: current vehicle position {north, east, down} (meters)
-- wpt_start_ned: previous waypoint {north, east, down} (meters)
-- wpt_end_ned: target waypoint {north, east, down} (meters)
-- lookahead_dist_m: LOS lookahead distance Delta (e.g. 5.0 to 15.0 meters)
-- Returns sequence {cmd_heading_rad, cmd_pitch_rad, cross_track_err_m, along_track_dist_m, reached_wpt}
public function los_guidance_step(
    sequence veh_pos_ned,
    sequence wpt_start_ned,
    sequence wpt_end_ned,
    atom lookahead_dist_m,
    atom acceptance_radius_m
)
    sequence path_vec = vec_sub(wpt_end_ned, wpt_start_ned)
    atom path_len_horiz = sqrt(path_vec[1]*path_vec[1] + path_vec[2]*path_vec[2])

    if path_len_horiz < 0.1 then
        path_len_horiz = 0.1
    end if

    -- Horizontal path azimuth angle
    atom psi_path = atan2(path_vec[2], path_vec[1])

    -- Relative position vector from start waypoint to vehicle
    sequence rel_vec = vec_sub(veh_pos_ned, wpt_start_ned)

    -- Along-track distance: projection onto path tangent
    atom s_along = rel_vec[1] * cos(psi_path) + rel_vec[2] * sin(psi_path)

    -- Cross-track horizontal error (positive means vehicle is to the right of path)
    atom e_cross = -rel_vec[1] * sin(psi_path) + rel_vec[2] * cos(psi_path)

    -- Vertical depth error along path (downwards is positive)
    atom path_progress = clamp(s_along / path_len_horiz, 0.0, 1.0)
    atom target_depth = wpt_start_ned[3] + path_progress * (wpt_end_ned[3] - wpt_start_ned[3])
    atom e_depth = target_depth - veh_pos_ned[3]

    -- LOS steering heading command: psi_cmd = psi_path + arctan(-e_cross / Delta)
    atom delta = max({lookahead_dist_m, 1.0})
    atom psi_los = psi_path + arctan(-e_cross / delta)
    psi_los = wrap_pi(psi_los)

    -- LOS vertical pitch command: theta_cmd = arctan(e_depth / Delta)
    atom theta_los = arctan(e_depth / delta)
    theta_los = clamp(theta_los, -30.0 * DEG2RAD, 30.0 * DEG2RAD)

    -- Check if vehicle has reached target waypoint within acceptance sphere
    atom dist_to_end = vec_norm(vec_sub(wpt_end_ned, veh_pos_ned))
    integer reached = 0
    if dist_to_end <= acceptance_radius_m or s_along >= path_len_horiz then
        reached = 1
    end if

    return {psi_los, theta_los, e_cross, s_along, reached}
end function
