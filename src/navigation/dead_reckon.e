-- Moonpool Subsea Robotics Sensor Fusion
-- Strapdown Dead-Reckoning Navigation and Kinematic State Propagation
-- OpenEuphoria v4.1+ Specification

namespace moonpool_dead_reckon

include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e
include src/core/quaternion.e

-- Create a dead-reckoning state structure:
-- {pos_ned, vel_ned, att_quat, elapsed_time_sec, total_distance_m}
public function dr_state_create(sequence pos_ned, sequence vel_ned, sequence att_quat)
    return {pos_ned, vel_ned, quat_normalize(att_quat), 0.0, 0.0}
end function

-- Propagate dead-reckoning state over time step dt
-- state: current DR state
-- v_body: vehicle body velocities {u, v, w} from DVL (m/s)
-- gyro_rates: angular velocities {p, q, r} from rate gyroscopes (rad/s)
-- dt_sec: time interval (seconds)
public function dr_propagate(sequence state, sequence v_body, sequence gyro_rates, atom dt_sec)
    sequence pos_ned = state[1]
    sequence vel_ned = state[2]
    sequence q = state[3]
    atom elapsed = state[4]
    atom distance = state[5]

    sequence R_b_n, v_nav, delta_pos
    sequence dq, q_new
    atom p = gyro_rates[1], q_y = gyro_rates[2], r = gyro_rates[3]
    atom omega_norm

    -- 1. Attitude propagation via quaternion kinematics:
    -- q_dot = 0.5 * q * [0, p, q_y, r]
    omega_norm = sqrt(p*p + q_y*q_y + r*r)
    if omega_norm * dt_sec > 1.0e-8 then
        atom half_angle = 0.5 * omega_norm * dt_sec
        atom s_half = sin(half_angle) / omega_norm
        dq = {cos(half_angle), s_half * p, s_half * q_y, s_half * r}
        q_new = quat_normalize(quat_mul(q, dq))
    else
        dq = {1.0, 0.5 * p * dt_sec, 0.5 * q_y * dt_sec, 0.5 * r * dt_sec}
        q_new = quat_normalize(quat_mul(q, dq))
    end if

    -- 2. Transform body velocities to navigation frame
    R_b_n = quat_to_dcm(q_new)
    v_nav = mat_vec_mul(R_b_n, v_body)

    -- 3. Position integration (trapezoidal / midpoint velocity)
    sequence v_avg = vec_scale(vec_add(vel_ned, v_nav), 0.5)
    delta_pos = vec_scale(v_avg, dt_sec)
    pos_ned = vec_add(pos_ned, delta_pos)
    vel_ned = v_nav

    elapsed += dt_sec
    distance += vec_norm(delta_pos)

    return {pos_ned, vel_ned, q_new, elapsed, distance}
end function

-- Estimate accumulated dead-reckoning position uncertainty circle radius (meters)
-- dt_elapsed: time since last absolute acoustic fix (seconds)
-- dist_traveled: total distance traveled since fix (meters)
-- dvl_error_pct: DVL distance scale factor error (e.g. 0.2% = 0.002)
-- gyro_drift_deg_hr: gyro bias drift in degrees/hour
public function dr_estimate_drift_radius(
    atom dt_elapsed,
    atom dist_traveled,
    atom dvl_error_pct,
    atom gyro_drift_deg_hr
)
    atom dist_error = dist_traveled * dvl_error_pct
    atom heading_drift_rad = (gyro_drift_deg_hr * DEG2RAD / 3600.0) * dt_elapsed
    atom cross_track_drift = dist_traveled * sin(heading_drift_rad)

    return sqrt(dist_error * dist_error + cross_track_drift * cross_track_drift)
end function
