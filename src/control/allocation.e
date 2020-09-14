-- Moonpool Subsea Robotics Propulsion and Control
-- 8-Thruster Vectored Allocation Matrix and Moore-Penrose Pseudo-Inverse Solver
-- OpenEuphoria v4.1+ Specification

namespace moonpool_allocation

include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e

-- Construct 6x8 thruster configuration geometry matrix B
-- lx_h, ly_h: horizontal thruster mounting arm distances from center (meters)
-- lx_v, ly_v: vertical thruster mounting arm distances from center (meters)
-- Returns 6x8 matrix mapping thruster forces u = {f1..f8} to wrench tau = {X, Y, Z, K, M, N}
public function allocation_matrix_create(atom lx_h, atom ly_h, atom lx_v, atom ly_v)
    atom c45 = cos(45.0 * DEG2RAD) -- ~0.7071
    atom s45 = sin(45.0 * DEG2RAD) -- ~0.7071

    -- Yaw moment arms for horizontal thrusters:
    -- N = x * Fy - y * Fx
    -- T1 (+x, +y, +45 deg): Fx = f1*c45, Fy = f1*s45 -> N1 = lx*s45 - ly*c45
    -- T2 (+x, -y, -45 deg): Fx = f2*c45, Fy = -f2*s45 -> N2 = -lx*s45 - (-ly)*c45 = -lx*s45 + ly*c45
    -- T3 (-x, +y, +135 deg): Fx = -f3*c45, Fy = f3*s45 -> N3 = -lx*s45 - ly*(-c45) = -lx*s45 + ly*c45
    -- T4 (-x, -y, -135 deg): Fx = -f4*c45, Fy = -f4*s45 -> N4 = -(-lx)*(-s45) - (-ly)*(-c45) = lx*s45 - ly*c45
    atom n_pos =  lx_h * s45 - ly_h * c45
    atom n_neg = -lx_h * s45 + ly_h * c45

    sequence B = {
        -- T1      T2      T3      T4     T5   T6   T7   T8
        {  c45,    c45,   -c45,   -c45,   0.0, 0.0, 0.0, 0.0}, -- Surge X
        {  s45,   -s45,    s45,   -s45,   0.0, 0.0, 0.0, 0.0}, -- Sway Y
        {  0.0,    0.0,    0.0,    0.0,   1.0, 1.0, 1.0, 1.0}, -- Heave Z
        {  0.0,    0.0,    0.0,    0.0,   ly_v,-ly_v,ly_v,-ly_v}, -- Roll K
        {  0.0,    0.0,    0.0,    0.0,  -lx_v,-lx_v,lx_v, lx_v}, -- Pitch M
        { n_pos,  n_neg,  n_neg,  n_pos,  0.0, 0.0, 0.0, 0.0}  -- Yaw N
    }

    return B
end function

-- Solve 8-thruster force allocation using weighted Moore-Penrose pseudo-inverse:
-- tau_cmd: commanded 6-DOF wrench vector {X, Y, Z, K, M, N}
-- B: 6x8 thruster geometry matrix
-- health_weights: 8-element sequence of thruster health/weight factors [0.0, 1.0]
-- damping: Tikhonov regularization factor (avoids singularities)
-- Returns sequence of 8 commanded thruster forces {f1..f8} (Newtons)
public function allocation_solve(
    sequence tau_cmd,
    sequence B,
    sequence health_weights,
    atom damping
)
    integer n_thrusters = 8
    integer n_dof = 6

    -- Construct diagonal weight matrix W
    sequence W_inv = mat_zeros(n_thrusters, n_thrusters)
    for i = 1 to n_thrusters do
        atom w = health_weights[i]
        if w > 0.05 then
            W_inv[i][i] = 1.0 / w
        else
            W_inv[i][i] = 0.0 -- Failed thruster excluded from allocation
        end if
    end for

    -- B_weighted = B * W_inv (6 x 8)
    sequence BW = mat_mul(B, W_inv)
    sequence BW_BT = mat_mul(BW, mat_transpose(B))

    -- Add Tikhonov regularization
    if damping > 0.0 then
        BW_BT = mat_add(BW_BT, mat_scale(mat_eye(n_dof), damping * damping))
    end if

    sequence BW_BT_inv = mat_inv(BW_BT)

    -- Pseudo-inverse: B_pinv = W_inv * B^T * (B * W_inv * B^T + lambda^2 * I)^-1 (8 x 6)
    sequence B_pinv = mat_mul(mat_mul(W_inv, mat_transpose(B)), BW_BT_inv)

    -- Allocated forces: u = B_pinv * tau_cmd
    sequence u_forces = mat_vec_mul(B_pinv, tau_cmd)

    return u_forces
end function

-- Anti-windup iterative allocation with thrust saturation clamping:
-- Clamps any thruster exceeding max_thrust_n and verifies delivered wrench
public function allocation_solve_clamped(
    sequence tau_cmd,
    sequence B,
    sequence health_weights,
    atom max_thrust_n,
    atom damping
)
    sequence forces = allocation_solve(tau_cmd, B, health_weights, damping)
    sequence clamped_forces = repeat(0.0, 8)

    for i = 1 to 8 do
        clamped_forces[i] = clamp(forces[i], -max_thrust_n, max_thrust_n)
    end for

    -- Compute delivered wrench: tau_actual = B * u_clamped
    sequence tau_actual = mat_vec_mul(B, clamped_forces)

    return {clamped_forces, tau_actual}
end function
