-- Moonpool Subsea Robotics Sensor Fusion
-- 15-State Error-State Extended Kalman Filter (ES-EKF) for Subsea Navigation
-- OpenEuphoria v4.1+ Specification

namespace moonpool_ekf

include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e
include src/core/quaternion.e

-- Create skew-symmetric matrix [v]x from 3D vector v:
-- Maps cross product: [v]x * u = v x u
public function skew_symmetric(sequence v)
    return {
        { 0.0,  -v[3],  v[2]},
        { v[3],  0.0,  -v[1]},
        {-v[2],  v[1],  0.0 }
    }
end function

-- Initialize 15-state ES-EKF structure:
-- Returns sequence {nominal_state, P_cov, Q_proc}
-- nominal_state = {pos_ned(3), vel_ned(3), quat(4), accel_bias(3), gyro_bias(3)}
public function ekf_create(
    sequence init_pos,
    sequence init_vel,
    sequence init_quat,
    atom pos_var,
    atom vel_var,
    atom att_var
)
    sequence nom_state
    sequence P, Q

    nom_state = {
        init_pos,               -- 1: pos_ned {n, e, d}
        init_vel,               -- 2: vel_ned {vn, ve, vd}
        quat_normalize(init_quat), -- 3: attitude quat {w, x, y, z}
        {0.0, 0.0, 0.0},        -- 4: accel_bias {bx, by, bz} (m/s^2)
        {0.0, 0.0, 0.0}         -- 5: gyro_bias {bgx, bgy, bgz} (rad/s)
    }

    -- 15x15 Error state covariance P
    P = mat_zeros(15, 15)
    for i = 1 to 3 do
        P[i][i] = pos_var
        P[i+3][i+3] = vel_var
        P[i+6][i+6] = att_var
        P[i+9][i+9] = 1.0e-4   -- Accel bias variance
        P[i+12][i+12] = 1.0e-6 -- Gyro bias variance
    end for

    -- 15x15 Process noise matrix Q
    Q = mat_zeros(15, 15)
    for i = 1 to 3 do
        Q[i][i] = 1.0e-5       -- Position drift noise
        Q[i+3][i+3] = 1.0e-3   -- Acceleration noise (m/s^2)^2
        Q[i+6][i+6] = 1.0e-4   -- Gyro angular rate noise (rad/s)^2
        Q[i+9][i+9] = 1.0e-7   -- Accel bias random walk
        Q[i+12][i+12] = 1.0e-9 -- Gyro bias random walk
    end for

    return {nom_state, P, Q}
end function

-- ES-EKF Kinematic Prediction Step using IMU specific force and angular rates
public function ekf_predict(sequence filter, sequence accel_raw, sequence gyro_raw, atom dt_sec)
    sequence nom = filter[1]
    sequence P = filter[2]
    sequence Q = filter[3]

    sequence pos = nom[1]
    sequence vel = nom[2]
    sequence q   = nom[3]
    sequence ba  = nom[4]
    sequence bg  = nom[5]

    sequence a_corr, omega_corr, R_b_n, f_nav, gravity_vec
    sequence Phi, F, P_new
    atom omega_mag

    -- 1. Compensate IMU measurements with current bias estimates
    a_corr = vec_sub(accel_raw, ba)
    omega_corr = vec_sub(gyro_raw, bg)

    -- 2. Propagate attitude quaternion
    R_b_n = quat_to_dcm(q)
    omega_mag = vec_norm(omega_corr)
    if omega_mag * dt_sec > 1.0e-8 then
        atom half_angle = 0.5 * omega_mag * dt_sec
        atom s_h = sin(half_angle) / omega_mag
        sequence dq = {cos(half_angle), s_h * omega_corr[1], s_h * omega_corr[2], s_h * omega_corr[3]}
        q = quat_normalize(quat_mul(q, dq))
    end if

    -- Update DCM after rotation
    R_b_n = quat_to_dcm(q)

    -- 3. Propagate velocity and position in NED frame
    gravity_vec = {0.0, 0.0, GRAVITY_STANDARD}
    f_nav = vec_add(mat_vec_mul(R_b_n, a_corr), gravity_vec)

    sequence vel_new = vec_add(vel, vec_scale(f_nav, dt_sec))
    sequence pos_new = vec_add(pos, vec_scale(vec_scale(vec_add(vel, vel_new), 0.5), dt_sec))

    -- 4. Construct 15x15 Error State Transition Matrix Phi = I + F * dt
    F = mat_zeros(15, 15)

    -- d(delta_p) / dt = delta_v
    for i = 1 to 3 do
        F[i][i+3] = 1.0
    end for

    -- d(delta_v) / dt = -R * [a_corr]x * delta_theta - R * delta_ba
    sequence a_skew = skew_symmetric(a_corr)
    sequence Ra_skew = mat_mul(R_b_n, a_skew)
    for i = 1 to 3 do
        for j = 1 to 3 do
            F[i+3][j+6] = -Ra_skew[i][j]
            F[i+3][j+9] = -R_b_n[i][j]
        end for
    end for

    -- d(delta_theta) / dt = -[omega_corr]x * delta_theta - delta_bg
    sequence w_skew = skew_symmetric(omega_corr)
    for i = 1 to 3 do
        for j = 1 to 3 do
            F[i+6][j+6] = -w_skew[i][j]
        end for
        F[i+6][i+12] = -1.0
    end for

    -- Discrete state transition Phi = I + F * dt
    Phi = mat_add(mat_eye(15), mat_scale(F, dt_sec))

    -- 5. Covariance propagation: P = Phi * P * Phi^T + Q * dt
    sequence Phi_P = mat_mul(Phi, P)
    P_new = mat_add(mat_mul(Phi_P, mat_transpose(Phi)), mat_scale(Q, dt_sec))

    nom[1] = pos_new
    nom[2] = vel_new
    nom[3] = q
    nom[4] = ba
    nom[5] = bg

    return {nom, P_new, Q}
end function

-- ES-EKF Measurement Update with 3D Absolute Position Fix (USBL or LBL)
public function ekf_update_position(sequence filter, sequence pos_meas_ned, atom pos_noise_var)
    sequence nom = filter[1]
    sequence P = filter[2]
    sequence Q = filter[3]

    sequence pos = nom[1]
    sequence vel = nom[2]
    sequence q   = nom[3]
    sequence ba  = nom[4]
    sequence bg  = nom[5]

    sequence H, HT, R_cov, S, S_inv, K, innov, delta_x, P_new
    integer m = 3

    -- Measurement matrix H (3 x 15): delta_z = delta_p
    H = mat_zeros(3, 15)
    for i = 1 to 3 do
        H[i][i] = 1.0
    end for

    -- Innovation: y = z_meas - p_pred
    innov = vec_sub(pos_meas_ned, pos)

    -- Innovation covariance: S = H * P * H^T + R
    HT = mat_transpose(H)
    R_cov = mat_scale(mat_eye(3), pos_noise_var)
    S = mat_add(mat_mul(H, mat_mul(P, HT)), R_cov)
    S_inv = mat_inv(S)

    -- Kalman gain: K = P * H^T * S^-1 (15 x 3)
    K = mat_mul(mat_mul(P, HT), S_inv)

    -- Error state correction: delta_x = K * innov (15)
    delta_x = mat_vec_mul(K, innov)

    -- Inject error state into nominal state
    pos = vec_add(pos, {delta_x[1], delta_x[2], delta_x[3]})
    vel = vec_add(vel, {delta_x[4], delta_x[5], delta_x[6]})

    -- Multiplicative quaternion correction: q = q * [1, 0.5 * delta_theta]
    sequence d_theta = {delta_x[7], delta_x[8], delta_x[9]}
    sequence dq = {1.0, 0.5 * d_theta[1], 0.5 * d_theta[2], 0.5 * d_theta[3]}
    q = quat_normalize(quat_mul(q, dq))

    ba = vec_add(ba, {delta_x[10], delta_x[11], delta_x[12]})
    bg = vec_add(bg, {delta_x[13], delta_x[14], delta_x[15]})

    -- Joseph-form covariance update: P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
    sequence I_KH = mat_sub(mat_eye(15), mat_mul(K, H))
    sequence P_temp = mat_mul(mat_mul(I_KH, P), mat_transpose(I_KH))
    sequence KRKT = mat_mul(mat_mul(K, R_cov), mat_transpose(K))
    P_new = mat_add(P_temp, KRKT)

    nom[1] = pos
    nom[2] = vel
    nom[3] = q
    nom[4] = ba
    nom[5] = bg

    return {nom, P_new, Q}
end function

-- ES-EKF Measurement Update with DVL Body-Frame 3D Velocities
public function ekf_update_dvl(sequence filter, sequence v_body_meas, atom dvl_noise_var)
    sequence nom = filter[1]
    sequence P = filter[2]
    sequence Q = filter[3]

    sequence pos = nom[1]
    sequence vel = nom[2]
    sequence q   = nom[3]
    sequence ba  = nom[4]
    sequence bg  = nom[5]

    sequence R_b_n, R_n_b, H, HT, R_cov, S, S_inv, K, innov, delta_x, P_new
    sequence v_pred_body

    R_b_n = quat_to_dcm(q)
    R_n_b = mat_transpose(R_b_n)

    -- Predicted body velocity: v_b = R_n_b * v_n
    v_pred_body = mat_vec_mul(R_n_b, vel)
    innov = vec_sub(v_body_meas, v_pred_body)

    -- H matrix: d(v_b)/d(delta_v) = R_n_b, d(v_b)/d(delta_theta) = [v_b]x
    H = mat_zeros(3, 15)
    sequence vb_skew = skew_symmetric(v_pred_body)
    for i = 1 to 3 do
        for j = 1 to 3 do
            H[i][j+3] = R_n_b[i][j]
            H[i][j+6] = vb_skew[i][j]
        end for
    end for

    HT = mat_transpose(H)
    R_cov = mat_scale(mat_eye(3), dvl_noise_var)
    S = mat_add(mat_mul(H, mat_mul(P, HT)), R_cov)
    S_inv = mat_inv(S)

    K = mat_mul(mat_mul(P, HT), S_inv)
    delta_x = mat_vec_mul(K, innov)

    pos = vec_add(pos, {delta_x[1], delta_x[2], delta_x[3]})
    vel = vec_add(vel, {delta_x[4], delta_x[5], delta_x[6]})

    sequence d_theta = {delta_x[7], delta_x[8], delta_x[9]}
    sequence dq = {1.0, 0.5 * d_theta[1], 0.5 * d_theta[2], 0.5 * d_theta[3]}
    q = quat_normalize(quat_mul(q, dq))

    ba = vec_add(ba, {delta_x[10], delta_x[11], delta_x[12]})
    bg = vec_add(bg, {delta_x[13], delta_x[14], delta_x[15]})

    sequence I_KH = mat_sub(mat_eye(15), mat_mul(K, H))
    sequence P_temp = mat_mul(mat_mul(I_KH, P), mat_transpose(I_KH))
    sequence KRKT = mat_mul(mat_mul(K, R_cov), mat_transpose(K))
    P_new = mat_add(P_temp, KRKT)

    nom[1] = pos
    nom[2] = vel
    nom[3] = q
    nom[4] = ba
    nom[5] = bg

    return {nom, P_new, Q}
end function

-- ES-EKF Measurement Update with 1D Pressure Depth Sensor
public function ekf_update_depth(sequence filter, atom depth_meas_m, atom depth_noise_var)
    sequence nom = filter[1]
    sequence P = filter[2]
    sequence Q = filter[3]

    sequence pos = nom[1]
    sequence vel = nom[2]
    sequence q   = nom[3]
    sequence ba  = nom[4]
    sequence bg  = nom[5]

    atom pred_depth = pos[3]
    atom innov = depth_meas_m - pred_depth

    -- H is 1x15 with 1 at index 3 (depth)
    atom s_scalar = P[3][3] + depth_noise_var
    if s_scalar < EPSILON then
        s_scalar = EPSILON
    end if
    atom inv_s = 1.0 / s_scalar

    -- Kalman gain vector K (15 x 1)
    sequence K = repeat(0.0, 15)
    for i = 1 to 15 do
        K[i] = P[i][3] * inv_s
    end for

    sequence delta_x = repeat(0.0, 15)
    for i = 1 to 15 do
        delta_x[i] = K[i] * innov
    end for

    pos = vec_add(pos, {delta_x[1], delta_x[2], delta_x[3]})
    vel = vec_add(vel, {delta_x[4], delta_x[5], delta_x[6]})

    sequence d_theta = {delta_x[7], delta_x[8], delta_x[9]}
    sequence dq = {1.0, 0.5 * d_theta[1], 0.5 * d_theta[2], 0.5 * d_theta[3]}
    q = quat_normalize(quat_mul(q, dq))

    ba = vec_add(ba, {delta_x[10], delta_x[11], delta_x[12]})
    bg = vec_add(bg, {delta_x[13], delta_x[14], delta_x[15]})

    -- Covariance update: P = P - K * H * P
    sequence P_new = P
    for i = 1 to 15 do
        for j = 1 to 15 do
            P_new[i][j] -= K[i] * P[3][j]
        end for
    end for

    nom[1] = pos
    nom[2] = vel
    nom[3] = q
    nom[4] = ba
    nom[5] = bg

    return {nom, P_new, Q}
end function
