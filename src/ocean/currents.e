-- Moonpool Subsea Robotics Oceanographic Physics
-- Ekman Spiral Current Profiles and Subsea Disturbance Flow
-- OpenEuphoria v4.1+ Specification

namespace moonpool_currents

include std/math.e
include src/core/constants.e
include src/core/math_util.e

-- Calculate wind-driven Ekman current velocity profile with depth:
-- wind_speed_m_s: 10m surface wind speed (m/s)
-- depth_z_m: depth below surface (positive downwards, meters)
-- lat_deg: latitude in degrees (determines Coriolis parameter f)
-- Returns sequence {u_east_m_s, v_north_m_s}
public function ekman_current_profile(atom wind_speed_m_s, atom depth_z_m, atom lat_deg)
    -- Surface current speed is approximately 2.5% to 3.0% of 10m wind speed
    atom v0 = 0.028 * wind_speed_m_s

    -- Coriolis parameter f = 2 * omega * sin(lat)
    atom f_coriolis = 2.0 * EARTH_ROTATION_RATE * sin(lat_deg * DEG2RAD)
    if abs(f_coriolis) < 1.0e-5 then
        f_coriolis = 1.0e-5
    end if

    -- Ekman depth of frictional influence D_E (meters)
    atom D_E = 45.0 * sqrt(max({1.0, wind_speed_m_s})) / sqrt(abs(f_coriolis) * 1.0e4)
    if D_E < 10.0 then
        D_E = 10.0
    end if

    -- Exponential decay: a = z / D_E
    atom a = depth_z_m / D_E
    atom decay = exp(-a)

    -- Deflection angle: 45 deg at surface, rotating clockwise with depth
    atom theta = (45.0 * DEG2RAD) + a

    atom u = v0 * decay * cos(theta)
    atom v = v0 * decay * sin(theta)

    return {u, v}
end function

-- Combined total subsea ocean current velocity vector at vehicle depth:
-- Combines background geostrophic flow, tidal stream, and Ekman surface layer
public function ocean_current_total(
    atom depth_z_m,
    sequence geostrophic_current_ned,
    atom tidal_stream_fwd,
    atom tidal_heading_rad,
    atom wind_speed_m_s,
    atom lat_deg
)
    sequence ekman = ekman_current_profile(wind_speed_m_s, depth_z_m, lat_deg)
    atom u_tide = tidal_stream_fwd * sin(tidal_heading_rad)
    atom v_tide = tidal_stream_fwd * cos(tidal_heading_rad)

    atom total_north = geostrophic_current_ned[1] + v_tide + ekman[2]
    atom total_east  = geostrophic_current_ned[2] + u_tide + ekman[1]
    atom total_down  = geostrophic_current_ned[3]

    return {total_north, total_east, total_down}
end function
