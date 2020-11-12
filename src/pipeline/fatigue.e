-- Moonpool Subsea Robotics Pipeline Engineering
-- Rainflow Cycle Counting, S-N Curves (DNV-RP-C203), and Palmgren-Miner Fatigue Damage
-- OpenEuphoria v4.1+ Specification

namespace moonpool_fatigue

include std/math.e
include src/core/constants.e
include src/core/math_util.e

-- DNV-RP-C203 S-N Curve Parameters for Seawater with Cathodic Protection
-- Curve D: log10(a) = 12.164, slope m1 = 3.0 (for N <= 10^7 cycles)
-- Slope m2 = 5.0 (for N > 10^7 cycles)
public constant SN_CURVE_D_LOG_A1 = 12.164
public constant SN_CURVE_D_M1     = 3.0
public constant SN_CURVE_D_M2     = 5.0
public constant SN_TRANSITION_N   = 1.0e7

-- Calculate permissible cycles to failure N for given stress range delta_sigma (MPa)
public function sn_cycles_to_failure(atom delta_sigma_mpa)
    if delta_sigma_mpa < 1.0 then
        return 1.0e12 -- Below fatigue endurance threshold
    end if

    -- High-stress regime: N1 = 10^(log_a1) / (delta_sigma)^m1
    atom log_a1 = SN_CURVE_D_LOG_A1
    atom a1 = power(10.0, log_a1)
    atom n1 = a1 / power(delta_sigma_mpa, SN_CURVE_D_M1)

    if n1 <= SN_TRANSITION_N then
        return n1
    else
        -- Dual-slope knee regime (m2 = 5.0)
        atom s_knee = power(a1 / SN_TRANSITION_N, 1.0 / SN_CURVE_D_M1)
        atom a2 = SN_TRANSITION_N * power(s_knee, SN_CURVE_D_M2)
        return a2 / power(delta_sigma_mpa, SN_CURVE_D_M2)
    end if
end function

-- Calculate Palmgren-Miner cumulative fatigue damage index D = sum(n_i / N_i):
-- stress_ranges_mpa: sequence of stress ranges {ds1, ds2, ...} (MPa)
-- cycle_counts: sequence of cycle counts {n1, n2, ...}
-- Returns dimensionless damage sum D (D >= 1.0 indicates fatigue failure crack initiation)
public function fatigue_miner_sum(sequence stress_ranges_mpa, sequence cycle_counts)
    atom total_damage = 0.0
    integer n = length(stress_ranges_mpa)

    for i = 1 to n do
        atom ds = stress_ranges_mpa[i]
        atom ni = cycle_counts[i]
        atom N_fail = sn_cycles_to_failure(ds)
        if N_fail > 0.0 then
            total_damage += ni / N_fail
        end if
    end for

    return total_damage
end function

-- Simplified ASTM E1049-85 Rainflow Cycle Counting on peak/valley stress sequence:
-- peaks_valleys: alternating local maxima and minima stress samples (MPa)
-- Returns sequence of extracted stress ranges {ds1, ds2, ...}
public function rainflow_count_ranges(sequence peaks_valleys)
    integer n = length(peaks_valleys)
    sequence ranges = {}

    if n < 3 then
        return ranges
    end if

    sequence stack = {}
    for i = 1 to n do
        stack = append(stack, peaks_valleys[i])
        while length(stack) >= 3 do
            integer s_len = length(stack)
            atom s0 = stack[s_len - 2]
            atom s1 = stack[s_len - 1]
            atom s2 = stack[s_len]

            atom r_x = abs(s1 - s2)
            atom r_y = abs(s0 - s1)

            if r_x >= r_y then
                -- Form a closed cycle with range r_y
                ranges = append(ranges, r_y)
                -- Remove s0 and s1, re-insert s2
                stack = stack[1 .. s_len - 3] & {s2}
            else
                exit
            end if
        end while
    end for

    return ranges
end function
