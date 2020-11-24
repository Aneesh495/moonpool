-- Moonpool Subsea Robotics Oceanographic Physics
-- Airy Linear Wave Theory, Dispersion Relation, and Subsea Orbital Velocities
-- OpenEuphoria v4.1+ Specification

namespace moonpool_waves

include std/math.e
include src/core/constants.e
include src/core/math_util.e

-- Solve linear wave dispersion relation for wavenumber k (rad/m):
-- omega^2 = g * k * tanh(k * h)
-- omega_rad_s: wave angular frequency 2*PI / T (rad/s)
-- water_depth_m: water depth h (meters)
public function wave_solve_wavenumber(atom omega_rad_s, atom water_depth_m)
    atom g = GRAVITY_STANDARD
    atom h = water_depth_m
    atom omega2 = omega_rad_s * omega_rad_s

    -- Deep water initial guess: k0 = omega^2 / g
    atom k0 = omega2 / g
    if k0 * h > 3.0 then
        return k0 -- Deep water regime
    end if

    -- Newton-Raphson iteration on dispersion equation
    atom k = k0
    for iter = 1 to 20 do
        atom th = tanh(k * h)
        atom f = g * k * th - omega2
        atom df = g * th + g * k * h * (1.0 - th * th)
        atom delta_k = f / df
        k -= delta_k
        if abs(delta_k) < 1.0e-6 then
            exit
        end if
    end for

    return k
end function

-- Calculate wave orbital horizontal and vertical velocities at subsea depth:
-- H_wave_m: wave height (crest-to-trough, meters)
-- T_wave_sec: wave period (seconds)
-- depth_z_m: depth below surface (positive downwards, meters)
-- water_depth_h_m: total seabed water depth (meters)
-- phase_rad: wave phase angle (radians)
-- Returns sequence {u_horiz_m_s, w_vert_m_s}
public function wave_orbital_velocities(
    atom H_wave_m,
    atom T_wave_sec,
    atom depth_z_m,
    atom water_depth_h_m,
    atom phase_rad
)
    atom omega = TWO_PI / T_wave_sec
    atom k = wave_solve_wavenumber(omega, water_depth_h_m)
    atom h = water_depth_h_m

    -- z coordinate is negative downwards in Airy notation: z_airy = -depth_z_m
    atom z_airy = -depth_z_m
    atom arg = k * (z_airy + h)
    if arg < 0.0 then
        arg = 0.0
    end if

    atom sinh_kh = sinh(k * h)
    if sinh_kh < 1.0e-5 then
        sinh_kh = 1.0e-5
    end if

    atom factor = 0.5 * H_wave_m * omega

    atom u_horiz = factor * (cosh(arg) / sinh_kh) * cos(phase_rad)
    atom w_vert  = factor * (sinh(arg) / sinh_kh) * sin(phase_rad)

    return {u_horiz, w_vert}
end function
