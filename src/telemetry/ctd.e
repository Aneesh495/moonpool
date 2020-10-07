-- Moonpool Subsea Robotics Telemetry Codecs
-- Sea-Bird CTD Protocol Processing and Practical Salinity Scale (PSS-78) Derivation
-- OpenEuphoria v4.1+ Specification

namespace moonpool_ctd

include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/acoustic/sound_speed.e

-- UNESCO Practical Salinity Scale 1978 (PSS-78) Standard Coefficients
constant A0 =  0.0080
constant A1 = -0.1692
constant A2 = 25.3851
constant A3 = 14.0941
constant A4 = -7.0261
constant A5 =  2.7081

constant B0 =  0.0005
constant B1 = -0.0056
constant B2 = -0.0066
constant B3 = -0.0375
constant B4 =  0.0636
constant B5 = -0.0144

constant C3515 = 4.2914 -- Conductivity of standard seawater at 35 PSU, 15 deg C (S/m)

-- Compute Practical Salinity (PSU) from conductivity (S/m), temperature (deg C), and pressure (dbar)
public function ctd_compute_salinity(atom cond_s_m, atom temp_c, atom press_dbar)
    atom r_t, rt_sqrt, s_val, delta_s
    atom c_ratio

    if cond_s_m <= 0.001 then
        return 0.0
    end if

    c_ratio = cond_s_m / C3515
    atom t = temp_c

    -- Temperature polynomial factor r_t
    atom c_t = 0.6766097 +
               2.00564e-2 * t +
               1.104259e-4 * (t * t) -
               6.9698e-7 * (t * t * t) +
               1.0031e-9 * (t * t * t * t)

    r_t = c_ratio / c_t
    if r_t < 0.0 then
        r_t = 0.0
    end if

    rt_sqrt = sqrt(r_t)

    s_val = A0 +
            A1 * rt_sqrt +
            A2 * r_t +
            A3 * (r_t * rt_sqrt) +
            A4 * (r_t * r_t) +
            A5 * (r_t * r_t * rt_sqrt)

    delta_s = ((t - 15.0) / (1.0 + 0.0162 * (t - 15.0))) * (
        B0 +
        B1 * rt_sqrt +
        B2 * r_t +
        B3 * (r_t * rt_sqrt) +
        B4 * (r_t * r_t) +
        B5 * (r_t * r_t * rt_sqrt)
    )

    return clamp(s_val + delta_s, 0.0, 42.0)
end function

-- Process raw CTD package measurement:
-- Returns sequence {salinity_psu, sound_speed_m_s, density_kg_m3}
public function ctd_process_packet(atom cond_s_m, atom temp_c, atom press_dbar)
    atom salinity = ctd_compute_salinity(cond_s_m, temp_c, press_dbar)
    atom depth_m = press_dbar * 0.992 -- Decibar to depth approximation
    atom c_sound = sound_speed_mackenzie(temp_c, salinity, depth_m)

    -- Empirical seawater density (UNESCO 1980 simplified equation of state)
    atom rho = 1000.0 + 28.14 - 0.0735 * temp_c + 0.802 * (salinity - 35.0)

    return {salinity, c_sound, rho}
end function
