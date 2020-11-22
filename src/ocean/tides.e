-- Moonpool Subsea Robotics Oceanographic Physics
-- Harmonic Tidal Elevation, Astronomical Constituents, and Tidal Currents
-- OpenEuphoria v4.1+ Specification

namespace moonpool_tides

include std/math.e
include src/core/constants.e
include src/core/math_util.e

-- Major North Sea Astronomical Tidal Constituent Frequencies (rad/sec)
public constant TIDE_OMEGA_M2 = 1.405189e-4 -- Principal lunar semidiurnal (12.42 hr)
public constant TIDE_OMEGA_S2 = 1.454441e-4 -- Principal solar semidiurnal (12.00 hr)
public constant TIDE_OMEGA_N2 = 1.378797e-4 -- Larger lunar elliptic (12.66 hr)
public constant TIDE_OMEGA_K1 = 7.292116e-5 -- Lunar diurnal (23.93 hr)
public constant TIDE_OMEGA_O1 = 6.759774e-5 -- Lunar diurnal (25.82 hr)

-- Create harmonic tidal constituent model for a subsea field:
-- constituents: sequence of {{omega_rad_s, amp_m, phase_deg}, ...}
public function tide_model_create(sequence constituent_list)
    return constituent_list
end function

-- Calculate astronomical tidal surface elevation (meters above LAT / MSL):
-- t_sec: elapsed time from epoch (seconds)
public function tide_surface_elevation(sequence tide_model, atom t_sec)
    atom eta = 0.0
    for i = 1 to length(tide_model) do
        sequence c = tide_model[i]
        atom omega = c[1]
        atom amp   = c[2]
        atom phase = c[3] * DEG2RAD
        eta += amp * cos(omega * t_sec - phase)
    end for
    return eta
end function

-- Calculate horizontal tidal stream current velocity (m/s) in North Sea channel:
-- max_current_m_s: peak spring tidal stream velocity (e.g. 1.2 m/s)
-- t_sec: elapsed time (seconds)
-- phase_offset_deg: local tidal stream slack water phase offset
public function tide_stream_velocity(atom max_current_m_s, atom t_sec, atom phase_offset_deg)
    atom phase = phase_offset_deg * DEG2RAD
    -- Dominated by principal M2 semidiurnal tide
    return max_current_m_s * sin(TIDE_OMEGA_M2 * t_sec - phase)
end function
