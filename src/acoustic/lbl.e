-- Moonpool Subsea Robotics Acoustic Navigation
-- Long Baseline (LBL) Multilateration and Seabed Transponder Array Least-Squares Solver
-- OpenEuphoria v4.1+ Specification

namespace moonpool_lbl

include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e

-- Solve 3D subsea position from seabed transponder range measurements
-- transponders: sequence of transponder NED positions {{n1, e1, d1}, {n2, e2, d2}, ...}
-- ranges: sequence of measured slant ranges {r1, r2, ...} (meters)
-- initial_guess: starting {north, east, down} estimate (meters)
-- tol_m: convergence tolerance (meters)
-- max_iter: maximum Gauss-Newton iterations
-- Returns sequence {pos_ned, residual_norm, dop_metrics, converged}
-- dop_metrics = {hdop, vdop, pdop}
public function lbl_solve_position(
    sequence transponders,
    sequence ranges,
    sequence initial_guess,
    atom tol_m,
    integer max_iter
)
    integer m, n_dim
    sequence p, J, res_vec, delta_p, JT, JTJ, JTJ_inv
    atom residual_norm, pred_r, diff_r
    atom damping = 1.0e-3
    integer converged = 0

    m = length(transponders)
    n_dim = 3

    if m < 3 or m != length(ranges) then
        return {initial_guess, 999.0, {99.0, 99.0, 99.0}, 0}
    end if

    p = initial_guess

    for iter = 1 to max_iter do
        J = mat_zeros(m, n_dim)
        res_vec = repeat(0.0, m)
        residual_norm = 0.0

        for i = 1 to m do
            sequence tp = transponders[i]
            sequence disp = vec_sub(p, tp)
            pred_r = vec_norm(disp)
            if pred_r < EPSILON then
                pred_r = EPSILON
            end if

            -- Unit direction vector (Jacobian row)
            J[i][1] = disp[1] / pred_r
            J[i][2] = disp[2] / pred_r
            J[i][3] = disp[3] / pred_r

            diff_r = ranges[i] - pred_r
            res_vec[i] = diff_r
            residual_norm += diff_r * diff_r
        end for

        residual_norm = sqrt(residual_norm / m)

        -- Normal equations: (J^T * J + lambda * I) * delta_p = J^T * res_vec
        JT = mat_transpose(J)
        JTJ = mat_mul(JT, J)
        if damping > 0.0 then
            JTJ = mat_add(JTJ, mat_scale(mat_eye(n_dim), damping))
        end if

        JTJ_inv = mat_inv(JTJ)
        delta_p = mat_vec_mul(JTJ_inv, mat_vec_mul(JT, res_vec))

        p = vec_add(p, delta_p)

        if vec_norm(delta_p) < tol_m then
            converged = 1
            exit
        end if
    end for

    -- Compute Dilution of Precision (DOP) metrics
    JT = mat_transpose(J)
    JTJ = mat_mul(JT, J)
    JTJ_inv = mat_inv(JTJ)

    atom q_nn = max({0.0, JTJ_inv[1][1]})
    atom q_ee = max({0.0, JTJ_inv[2][2]})
    atom q_dd = max({0.0, JTJ_inv[3][3]})

    atom hdop = sqrt(q_nn + q_ee)
    atom vdop = sqrt(q_dd)
    atom pdop = sqrt(q_nn + q_ee + q_dd)

    return {p, residual_norm, {hdop, vdop, pdop}, converged}
end function

-- Identify and isolate an outlier transponder based on normalized residual error
public function lbl_detect_outlier(sequence transponders, sequence ranges, sequence pos_est, atom threshold_m)
    integer m = length(transponders)
    integer outlier_idx = 0
    atom max_err = 0.0

    for i = 1 to m do
        atom pred = vec_norm(vec_sub(pos_est, transponders[i]))
        atom err = abs(ranges[i] - pred)
        if err > max_err then
            max_err = err
            if err > threshold_m then
                outlier_idx = i
            end if
        end if
    end for

    return {outlier_idx, max_err}
end function
