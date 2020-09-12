-- Moonpool Subsea Robotics Propulsion and Control
-- Marine Ducted Propeller Thrust Dynamics, Deadband, and Power Consumption
-- OpenEuphoria v4.1+ Specification

namespace moonpool_thruster_model

include std/math.e
include src/core/constants.e
include src/core/math_util.e

-- Create a marine thruster parameter structure:
-- max_fwd_n: maximum forward thrust (N, e.g. +350.0 N)
-- max_rev_n: maximum reverse thrust (N, e.g. -240.0 N)
-- rate_limit_n_s: maximum slew rate of thrust change (N/s, e.g. 500 N/s)
-- deadband_n: minimum threshold force below which motor does not spin (N, e.g. 5.0 N)
public function thruster_create(atom max_fwd_n, atom max_rev_n, atom rate_limit_n_s, atom deadband_n)
    return {max_fwd_n, max_rev_n, rate_limit_n_s, deadband_n}
end function

-- Calculate actual delivered thrust force after applying motor dynamics,
-- deadband, slew rate limiting, and asymmetric forward/reverse saturation
public function thruster_step_force(
    sequence thruster,
    atom cmd_n,
    atom current_n,
    atom dt_sec
)
    atom max_fwd = thruster[1]
    atom max_rev = abs(thruster[2])
    atom rate_limit = thruster[3]
    atom deadband = thruster[4]

    -- 1. Deadband filter
    atom target_n = cmd_n
    if abs(target_n) < deadband then
        target_n = 0.0
    end if

    -- 2. Slew rate limiter (motor inertia and ESC ramp)
    atom max_delta = rate_limit * dt_sec
    atom delta = target_n - current_n
    delta = clamp(delta, -max_delta, max_delta)
    atom next_n = current_n + delta

    -- 3. Asymmetric saturation clamp
    if next_n > max_fwd then
        next_n = max_fwd
    elsif next_n < -max_rev then
        next_n = -max_rev
    end if

    return next_n
end function

-- Estimate electrical power consumed by brushless DC thruster motor (Watts):
-- P = |T|^(1.5) / prop_efficiency + idle_power
public function thruster_power_watts(atom thrust_n)
    atom abs_f = abs(thrust_n)
    if abs_f < 0.1 then
        return 12.0 -- Quiescent ESC inverter idle power
    end if
    -- Empirical marine duct propeller power scaling factor
    atom hyd_power = power(abs_f, 1.5) * 0.42
    return hyd_power + 15.0
end function
