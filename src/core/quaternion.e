-- Moonpool Subsea Robotics Core
-- Unit Quaternions, 3D Spatial Attitude Kinematics, and Direction Cosine Matrices
-- OpenEuphoria v4.1+ Specification

namespace moonpool_quaternion

include std/math.e
include src/core/constants.e
include src/core/matrix.e

-- Create a quaternion sequence {w, x, y, z}
public function quat_create(atom w, atom x, atom y, atom z)
    return {w, x, y, z}
end function

-- Identity quaternion {1, 0, 0, 0}
public function quat_identity()
    return {1.0, 0.0, 0.0, 0.0}
end function

-- Quaternion Euclidean norm
public function quat_norm(sequence q)
    return sqrt(q[1]*q[1] + q[2]*q[2] + q[3]*q[3] + q[4]*q[4])
end function

-- Normalize quaternion to unit length
public function quat_normalize(sequence q)
    atom n
    n = quat_norm(q)
    if n < EPSILON then
        return quat_identity()
    end if
    return {q[1]/n, q[2]/n, q[3]/n, q[4]/n}
end function

-- Quaternion conjugate: q* = {w, -x, -y, -z}
public function quat_conjugate(sequence q)
    return {q[1], -q[2], -q[3], -q[4]}
end function

-- Quaternion Hamilton multiplication: q1 * q2
public function quat_mul(sequence q1, sequence q2)
    atom w1 = q1[1], x1 = q1[2], y1 = q1[3], z1 = q1[4]
    atom w2 = q2[1], x2 = q2[2], y2 = q2[3], z2 = q2[4]

    return {
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    }
end function

-- Construct quaternion from Tait-Bryan Euler angles: Roll (phi), Pitch (theta), Yaw (psi)
-- Rotations applied in Z-Y-X (Yaw-Pitch-Roll) sequence
public function quat_from_euler(atom roll, atom pitch, atom yaw)
    atom cr, sr, cp, sp, cy, sy
    atom half_roll = roll * 0.5
    atom half_pitch = pitch * 0.5
    atom half_yaw = yaw * 0.5

    cr = cos(half_roll)
    sr = sin(half_roll)
    cp = cos(half_pitch)
    sp = sin(half_pitch)
    cy = cos(half_yaw)
    sy = sin(half_yaw)

    return {
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    }
end function

-- Extract Euler angles {roll, pitch, yaw} in radians from unit quaternion
public function quat_to_euler(sequence q)
    atom w = q[1], x = q[2], y = q[3], z = q[4]
    atom roll, pitch, yaw
    atom sin_pitch

    -- Roll (x-axis rotation)
    atom sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    atom cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

    -- Pitch (y-axis rotation) with gimbal lock protection
    sin_pitch = 2.0 * (w * y - z * x)
    if abs(sin_pitch) >= 1.0 then
        pitch = HALF_PI * (sin_pitch / abs(sin_pitch))
    else
        pitch = arcsin(sin_pitch)
    end if

    -- Yaw (z-axis rotation)
    atom sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
    atom cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)

    return {roll, pitch, yaw}
end function

-- Convert quaternion to 3x3 Direction Cosine Matrix (DCM / Rotation Matrix R_b^n)
-- Maps vectors from vehicle body-fixed frame to North-East-Down navigation frame: v_n = R * v_b
public function quat_to_dcm(sequence q)
    atom w = q[1], x = q[2], y = q[3], z = q[4]
    atom x2 = x * x, y2 = y * y, z2 = z * z
    atom xy = x * y, xz = x * z, yz = y * z
    atom wx = w * x, wy = w * y, wz = w * z

    return {
        {1.0 - 2.0 * (y2 + z2), 2.0 * (xy - wz),       2.0 * (xz + wy)},
        {2.0 * (xy + wz),       1.0 - 2.0 * (x2 + z2), 2.0 * (yz - wx)},
        {2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (x2 + y2)}
    }
end function

-- Rotate a 3D vector v by quaternion q: v' = q * {0, v} * q*
public function quat_rotate_vec(sequence q, sequence v)
    sequence R
    R = quat_to_dcm(q)
    return mat_vec_mul(R, v)
end function

-- Spherical Linear Interpolation (SLERP) between q1 and q2 with blend factor t [0, 1]
public function quat_slerp(sequence q1, sequence q2, atom t)
    atom dot_val, theta, sin_theta, ratio_a, ratio_b
    sequence q2_adj = q2

    dot_val = q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3] + q1[4]*q2[4]

    -- If dot is negative, slerp won't take the shortest path
    if dot_val < 0.0 then
        q2_adj = {-q2[1], -q2[2], -q2[3], -q2[4]}
        dot_val = -dot_val
    end if

    if dot_val > 0.9995 then
        return quat_normalize({
            q1[1] + t * (q2_adj[1] - q1[1]),
            q1[2] + t * (q2_adj[2] - q1[2]),
            q1[3] + t * (q2_adj[3] - q1[3]),
            q1[4] + t * (q2_adj[4] - q1[4])
        })
    end if

    theta = arccos(dot_val)
    sin_theta = sin(theta)
    ratio_a = sin((1.0 - t) * theta) / sin_theta
    ratio_b = sin(t * theta) / sin_theta

    return {
        ratio_a * q1[1] + ratio_b * q2_adj[1],
        ratio_a * q1[2] + ratio_b * q2_adj[2],
        ratio_a * q1[3] + ratio_b * q2_adj[3],
        ratio_a * q1[4] + ratio_b * q2_adj[4]
    }
end function
