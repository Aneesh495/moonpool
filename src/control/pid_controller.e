-- Moonpool Subsea Robotics Propulsion and Control
-- 6-DOF Decoupled Trajectory Tracking PID Controller with Anti-Windup Protection
-- OpenEuphoria v4.1+ Specification

namespace moonpool_pid

include std/math.e
include src/core/constants.e
include src/core/math_util.e

-- Create a single-axis PID controller structure:
-- {kp, ki, kd, integral_accumulator, prev_measurement, int_limit, out_limit, is_initialized}
public function pid_axis_create(
    atom kp,
    atom ki,
    atom kd,
    atom int_limit,
    atom out_limit
)
    return {kp, ki, kd, 0.0, 0.0, int_limit, out_limit, 0}
end function

-- Update single PID axis:
-- setpoint: target state value
-- measurement: current state measurement
-- dt_sec: time interval (seconds)
-- Returns sequence {output_cmd, updated_axis_state}
public function pid_axis_step(
    sequence axis,
    atom setpoint,
    atom measurement,
    atom dt_sec
)
    atom kp = axis[1]
    atom ki = axis[2]
    atom kd = axis[3]
    atom integ = axis[4]
    atom prev_m = axis[5]
    atom int_limit = axis[6]
    atom out_limit = axis[7]
    integer initialized = axis[8]

    atom error = setpoint - measurement

    -- Proportional term
    atom P = kp * error

    -- Integral term with anti-windup clamping
    integ = clamp(integ + ki * error * dt_sec, -int_limit, int_limit)
    atom I = integ

    -- Derivative on measurement to prevent derivative step kick
    atom D = 0.0
    if initialized = 1 and dt_sec > 1.0e-5 then
        D = -kd * ((measurement - prev_m) / dt_sec)
    end if

    -- Total output command
    atom output_cmd = clamp(P + I + D, -out_limit, out_limit)

    axis[4] = integ
    axis[5] = measurement
    axis[8] = 1 -- Marked initialized

    return {output_cmd, axis}
end function

-- Create full 6-DOF decoupled controller:
-- gains: sequence of 6 axis specifications {{kp, ki, kd, int_lim, out_lim}, ...}
-- for {Surge, Sway, Heave, Roll, Pitch, Yaw}
public function pid_controller_6dof_create(sequence gains_6dof)
    sequence axes = repeat(0, 6)
    for i = 1 to 6 do
        sequence g = gains_6dof[i]
        axes[i] = pid_axis_create(g[1], g[2], g[3], g[4], g[5])
    end for
    return axes
end function

-- Step full 6-DOF controller:
-- setpoints: {X_sp, Y_sp, Z_sp, Roll_sp, Pitch_sp, Yaw_sp}
-- measurements: {X_m, Y_m, Z_m, Roll_m, Pitch_m, Yaw_m}
-- Returns sequence {tau_cmd_6dof, updated_controller}
public function pid_controller_6dof_step(
    sequence controller,
    sequence setpoints,
    sequence measurements,
    atom dt_sec
)
    sequence tau_cmd = repeat(0.0, 6)
    sequence new_controller = controller

    for i = 1 to 6 do
        atom sp = setpoints[i]
        atom meas = measurements[i]

        -- Special angle wrap handling for Roll, Pitch, Yaw
        if i >= 4 then
            atom err_angle = wrap_pi(sp - meas)
            sp = meas + err_angle
        end if

        sequence res = pid_axis_step(controller[i], sp, meas, dt_sec)
        tau_cmd[i] = res[1]
        new_controller[i] = res[2]
    end for

    return {tau_cmd, new_controller}
end function
