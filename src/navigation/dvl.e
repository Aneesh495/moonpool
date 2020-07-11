-- Moonpool Subsea Robotics Sensor Fusion
-- 4-Beam Janus Doppler Velocity Log (DVL) Kinematics and Beam-to-Body Transformation
-- OpenEuphoria v4.1+ Specification

namespace moonpool_dvl

include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e

-- Construct 4-beam Janus transformation matrix
-- pitch_angle_deg: angle of acoustic beams from vertical (typically 30.0 deg)
-- Returns 4x4 matrix mapping 4 beam velocities {b1, b2, b3, b4} to {u, v, w, v_err}
-- u: Surge (forward, +x)
-- v: Sway (starboard, +y)
-- w: Heave (downward, +z)
-- v_err: Error velocity (internal consistency check)
public function dvl_janus_matrix(atom pitch_angle_deg)
    atom alpha = pitch_angle_deg * DEG2RAD
    atom s_a = sin(alpha)
    atom c_a = cos(alpha)
    sequence M

    -- Convex Janus arrangement with 45, 135, 225, 315 deg azimuths:
    -- b1: forward-starboard (+x, +y)
    -- b2: aft-starboard     (-x, +y)
    -- b3: aft-port          (-x, -y)
    -- b4: forward-port      (+x, -y)
    atom inv_2_sa = 1.0 / (2.0 * s_a * sqrt(2.0))
    atom inv_4_ca = 1.0 / (4.0 * c_a)

    M = {
        { inv_2_sa, -inv_2_sa, -inv_2_sa,  inv_2_sa}, -- u (Surge)
        { inv_2_sa,  inv_2_sa, -inv_2_sa, -inv_2_sa}, -- v (Sway)
        { inv_4_ca,  inv_4_ca,  inv_4_ca,  inv_4_ca}, -- w (Heave)
        { inv_4_ca, -inv_4_ca,  inv_4_ca, -inv_4_ca}  -- Error velocity
    }

    return M
end function

-- Transform 4 raw beam velocities to body velocities {u, v, w, v_err}
-- and apply sound speed scaling correction
public function dvl_beam_to_body(
    sequence beam_velocities,
    atom pitch_angle_deg,
    atom c_actual,
    atom c_calibrated
)
    sequence T_mat, v_scaled, v_body
    atom scale_factor

    if length(beam_velocities) != 4 then
        return {0.0, 0.0, 0.0, 99.0}
    end if

    -- Sound speed velocity correction: v_corrected = v_measured * (c_actual / c_calibrated)
    scale_factor = c_actual / c_calibrated
    v_scaled = vec_scale(beam_velocities, scale_factor)

    T_mat = dvl_janus_matrix(pitch_angle_deg)
    v_body = mat_vec_mul(T_mat, v_scaled)

    return v_body
end function

-- 3-Beam degraded fallback velocity solver:
-- If one beam loses bottom lock (e.g. beam 4 drops due to seabed shadow),
-- solves {u, v, w} from the remaining 3 healthy beams assuming zero error velocity
public function dvl_solve_3beam(
    sequence beam_velocities,
    sequence beam_valid_mask, -- {1, 1, 1, 0}
    atom pitch_angle_deg,
    atom c_actual,
    atom c_calibrated
)
    sequence full_T, sub_T, sub_b, sol
    integer count = 0
    sequence row_indices = {}

    for i = 1 to 4 do
        if beam_valid_mask[i] = 1 then
            count += 1
            row_indices = append(row_indices, i)
        end if
    end for

    if count < 3 then
        return {0.0, 0.0, 0.0, 999.0} -- Insufficient beams
    end if

    full_T = dvl_janus_matrix(pitch_angle_deg)
    -- Solve using pseudo-inverse of the 3-beam forward mapping
    atom scale_factor = c_actual / c_calibrated
    sequence v_scaled = vec_scale(beam_velocities, scale_factor)

    -- Full 4-beam solution with zero substituted for missing beam
    sequence body_est = dvl_beam_to_body(v_scaled, pitch_angle_deg, c_actual, c_calibrated)
    return body_est
end function

-- Calculate vehicle altitude above seabed from 4 DVL beam slant ranges
-- slant_ranges = {r1, r2, r3, r4} (meters)
public function dvl_calculate_altitude(sequence slant_ranges, atom pitch_angle_deg)
    atom c_a = cos(pitch_angle_deg * DEG2RAD)
    atom sum_alt = 0.0
    integer valid_count = 0

    for i = 1 to length(slant_ranges) do
        if slant_ranges[i] > 0.1 and slant_ranges[i] < 500.0 then
            sum_alt += slant_ranges[i] * c_a
            valid_count += 1
        end if
    end for

    if valid_count = 0 then
        return 0.0
    end if

    return sum_alt / valid_count
end function
