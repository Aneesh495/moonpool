-- Moonpool Subsea Robotics Sensor Fusion
-- High-Precision Oceanographic Pressure-to-Depth Conversion and Latitude Gravitational Correction
-- OpenEuphoria v4.1+ Specification

namespace moonpool_depth_sensor

include std/math.e
include src/core/constants.e
include src/core/math_util.e

-- Calculate gravitational acceleration as a function of geodetic latitude (Somigliana's formula)
-- lat_deg: latitude in degrees (-90 to +90)
-- Returns local gravity g(phi) in m/s^2
public function gravity_latitude(atom lat_deg)
    atom phi = lat_deg * DEG2RAD
    atom s = sin(phi)
    atom s2 = s * s
    atom s4 = s2 * s2

    return 9.780318 * (1.0 + 5.2788e-3 * s2 + 2.36e-5 * s4)
end function

-- Convert hydrostatic gauge pressure to true depth using the Saunders and Fofonoff (1976) UNESCO standard:
-- pressure_dbar: ocean pressure in decibars (1 dbar approx 1 meter depth)
-- lat_deg: latitude in degrees
-- Returns true depth in meters below sea level
public function pressure_to_depth_saunders(atom pressure_dbar, atom lat_deg)
    atom p = pressure_dbar
    atom g_local = gravity_latitude(lat_deg)
    atom depth_std, depth_true

    -- Standard depth polynomial for 35 PSU standard ocean
    depth_std = (9.72659 * p -
                 2.2512e-5 * (p * p) +
                 2.279e-10 * (p * p * p) -
                 1.82e-15 * (p * p * p * p)) / 9.8

    -- Latitude gravitational correction factor
    depth_true = depth_std * (9.8 / g_local)

    return depth_true
end function

-- Calculate linear hydrostatic depth from absolute pressure in Pascals
-- abs_pressure_pa: total pressure measured by sensor (Pa)
-- atm_pressure_pa: atmospheric surface pressure (Pa, standard 101325 Pa)
-- density: average seawater density (kg/m^3, standard 1025 kg/m^3)
-- lat_deg: latitude in degrees
public function pressure_to_depth_linear(atom abs_pressure_pa, atom atm_pressure_pa, atom density, atom lat_deg)
    atom gauge_pa = abs_pressure_pa - atm_pressure_pa
    if gauge_pa < 0.0 then
        gauge_pa = 0.0
    end if

    atom g_local = gravity_latitude(lat_deg)
    return gauge_pa / (density * g_local)
end function

-- Calculate vertical heave velocity from sequential depth observations
public function depth_heave_velocity(atom depth_prev_m, atom depth_curr_m, atom dt_sec)
    if dt_sec < EPSILON then
        return 0.0
    end if
    -- In NED frame, Down is positive; increasing depth means positive heave velocity
    return (depth_curr_m - depth_prev_m) / dt_sec
end function
