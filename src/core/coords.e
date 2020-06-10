-- Moonpool Subsea Robotics Core
-- Geodetic Coordinate Reference Frames (WGS-84, ECEF, and Local NED Tangent Plane)
-- OpenEuphoria v4.1+ Specification

namespace moonpool_coords

include std/math.e
include src/core/constants.e
include src/core/math_util.e

-- Convert Geodetic (Latitude, Longitude in degrees, Altitude in meters) to ECEF {X, Y, Z}
public function geodetic_to_ecef(atom lat_deg, atom lon_deg, atom alt_m)
    atom phi, lambda_lon, sin_phi, cos_phi, sin_lambda, cos_lambda
    atom N_rad, x, y, z

    phi = lat_deg * DEG2RAD
    lambda_lon = lon_deg * DEG2RAD

    sin_phi = sin(phi)
    cos_phi = cos(phi)
    sin_lambda = sin(lambda_lon)
    cos_lambda = cos(lambda_lon)

    -- Prime vertical radius of curvature
    N_rad = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi)

    x = (N_rad + alt_m) * cos_phi * cos_lambda
    y = (N_rad + alt_m) * cos_phi * sin_lambda
    z = (N_rad * (1.0 - WGS84_E2) + alt_m) * sin_phi

    return {x, y, z}
end function

-- Convert ECEF {X, Y, Z} to Geodetic {Latitude deg, Longitude deg, Altitude m}
-- Implements Bowring's closed-form algorithm
public function ecef_to_geodetic(atom x, atom y, atom z)
    atom p, theta, sin_theta, cos_theta, phi, sin_phi
    atom N_rad, alt_m, lat_deg, lon_deg

    p = sqrt(x * x + y * y)
    if p < EPSILON then
        if z > 0.0 then
            return {90.0, 0.0, z - WGS84_B}
        else
            return {-90.0, 0.0, -z - WGS84_B}
        end if
    end if

    theta = atan2(z * WGS84_A, p * WGS84_B)
    sin_theta = sin(theta)
    cos_theta = cos(theta)

    phi = atan2(
        z + WGS84_E_PRIME2 * WGS84_B * sin_theta * sin_theta * sin_theta,
        p - WGS84_E2 * WGS84_A * cos_theta * cos_theta * cos_theta
    )

    sin_phi = sin(phi)
    N_rad = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi)
    alt_m = p / cos(phi) - N_rad

    lat_deg = phi * RAD2DEG
    lon_deg = atan2(y, x) * RAD2DEG

    return {lat_deg, lon_deg, alt_m}
end function

-- Convert ECEF coordinates to Local North-East-Down (NED) frame
-- relative to reference origin {lat0_deg, lon0_deg, alt0_m}
public function ecef_to_ned(sequence pos_ecef, atom lat0_deg, atom lon0_deg, atom alt0_m)
    sequence origin_ecef
    atom dx, dy, dz
    atom phi, lambda_lon, s_phi, c_phi, s_lam, c_lam
    atom north, east, down

    origin_ecef = geodetic_to_ecef(lat0_deg, lon0_deg, alt0_m)
    dx = pos_ecef[1] - origin_ecef[1]
    dy = pos_ecef[2] - origin_ecef[2]
    dz = pos_ecef[3] - origin_ecef[3]

    phi = lat0_deg * DEG2RAD
    lambda_lon = lon0_deg * DEG2RAD
    s_phi = sin(phi)
    c_phi = cos(phi)
    s_lam = sin(lambda_lon)
    c_lam = cos(lambda_lon)

    north = -s_phi * c_lam * dx - s_phi * s_lam * dy + c_phi * dz
    east  = -s_lam * dx         + c_lam * dy
    down  = -c_phi * c_lam * dx - c_phi * s_lam * dy - s_phi * dz

    return {north, east, down}
end function

-- Convert Local North-East-Down (NED) coordinates to ECEF frame
public function ned_to_ecef(sequence pos_ned, atom lat0_deg, atom lon0_deg, atom alt0_m)
    sequence origin_ecef
    atom n = pos_ned[1], e = pos_ned[2], d = pos_ned[3]
    atom phi, lambda_lon, s_phi, c_phi, s_lam, c_lam
    atom dx, dy, dz

    origin_ecef = geodetic_to_ecef(lat0_deg, lon0_deg, alt0_m)
    phi = lat0_deg * DEG2RAD
    lambda_lon = lon0_deg * DEG2RAD
    s_phi = sin(phi)
    c_phi = cos(phi)
    s_lam = sin(lambda_lon)
    c_lam = cos(lambda_lon)

    dx = -s_phi * c_lam * n - s_lam * e - c_phi * c_lam * d
    dy = -s_phi * s_lam * n + c_lam * e - c_phi * s_lam * d
    dz =  c_phi * n                     - s_phi * d

    return {origin_ecef[1] + dx, origin_ecef[2] + dy, origin_ecef[3] + dz}
end function

-- Great-circle distance between two geodetic coordinates in meters (Haversine formula)
public function haversine_distance(atom lat1_deg, atom lon1_deg, atom lat2_deg, atom lon2_deg)
    atom dlat, dlon, a, c
    dlat = (lat2_deg - lat1_deg) * DEG2RAD
    dlon = (lon2_deg - lon1_deg) * DEG2RAD

    a = sin(dlat * 0.5) * sin(dlat * 0.5) +
        cos(lat1_deg * DEG2RAD) * cos(lat2_deg * DEG2RAD) *
        sin(dlon * 0.5) * sin(dlon * 0.5)
    c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a))

    return WGS84_A * c
end function

-- Forward azimuth / bearing from point 1 to point 2 in degrees [0, 360)
public function bearing_deg(atom lat1_deg, atom lon1_deg, atom lat2_deg, atom lon2_deg)
    atom phi1, phi2, delta_lon, y, x, brg
    phi1 = lat1_deg * DEG2RAD
    phi2 = lat2_deg * DEG2RAD
    delta_lon = (lon2_deg - lon1_deg) * DEG2RAD

    y = sin(delta_lon) * cos(phi2)
    x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(delta_lon)
    brg = atan2(y, x) * RAD2DEG

    return wrap_360(brg)
end function
