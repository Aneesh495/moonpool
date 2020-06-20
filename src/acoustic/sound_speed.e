-- Moonpool Subsea Robotics Acoustic Navigation
-- Empirical Sound Velocity Profile (SVP) Modeling and Stratified Water Column Analysis
-- OpenEuphoria v4.1+ Specification

namespace moonpool_sound_speed

include std/math.e
include src/core/constants.e
include src/core/math_util.e

-- Calculate speed of sound in seawater using the Mackenzie (1981) formula:
-- Temperature T in degrees Celsius (2 to 30 deg C)
-- Salinity S in Practical Salinity Units (PSU, 25 to 40)
-- Depth D in meters (0 to 8000 m)
public function sound_speed_mackenzie(atom temp_c, atom salinity_psu, atom depth_m)
    atom c
    atom t = temp_c
    atom s = salinity_psu
    atom d = depth_m

    c = 1448.96 +
        4.591 * t -
        5.304e-2 * (t * t) +
        2.374e-4 * (t * t * t) +
        1.340 * (s - 35.0) +
        1.630e-2 * d +
        1.675e-7 * (d * d) -
        1.025e-2 * t * (s - 35.0) -
        7.139e-13 * t * (d * d * d)

    return c
end function

-- Calculate speed of sound in seawater using the Medwin (1975) equation:
-- Simplified formula valid for shallow water columns
public function sound_speed_medwin(atom temp_c, atom salinity_psu, atom depth_m)
    atom t = temp_c
    atom s = salinity_psu
    atom d = depth_m

    return 1449.2 +
           4.6 * t -
           0.055 * (t * t) +
           0.00029 * (t * t * t) +
           (1.34 - 0.01 * t) * (s - 35.0) +
           0.016 * d
end function

-- Calculate sound speed gradient dc/dz across a layer
-- layer = {{z1, c1}, {z2, c2}}
public function layer_gradient(sequence layer_top, sequence layer_bottom)
    atom dz = layer_bottom[1] - layer_top[1]
    if abs(dz) < EPSILON then
        return 0.0
    end if
    return (layer_bottom[2] - layer_top[2]) / dz
end function

-- Create a standard Munk sound velocity profile canonical model:
-- z: depth in meters
-- z_axis: depth of sound channel axis (e.g. 1000m)
-- c_axis: minimum sound speed at axis (e.g. 1492 m/s)
-- B: scale depth (e.g. 1000m), epsilon_munk: perturbation coefficient (0.00737)
public function sound_speed_munk(atom z, atom z_axis, atom c_axis, atom scale_b, atom eps_munk)
    atom eta, c
    eta = 2.0 * (z - z_axis) / scale_b
    c = c_axis * (1.0 + eps_munk * (eta + exp(-eta) - 1.0))
    return c
end function

-- Interpolate sound speed at arbitrary depth z from a discrete SVP profile:
-- profile = {{z_0, c_0}, {z_1, c_1}, ..., {z_n, c_n}} sorted by ascending depth
public function svp_interpolate(sequence profile, atom depth_m)
    integer n
    atom z0, z1, c0, c1, ratio

    n = length(profile)
    if n = 0 then
        return SOUND_SPEED_DEFAULT
    elsif n = 1 then
        return profile[1][2]
    end if

    -- Surface clamp
    if depth_m <= profile[1][1] then
        return profile[1][2]
    end if

    -- Seabed clamp
    if depth_m >= profile[n][1] then
        return profile[n][2]
    end if

    -- Layer search
    for i = 1 to n - 1 do
        z0 = profile[i][1]
        z1 = profile[i + 1][1]
        if depth_m >= z0 and depth_m <= z1 then
            c0 = profile[i][2]
            c1 = profile[i + 1][2]
            ratio = (depth_m - z0) / (z1 - z0)
            return lerp(c0, c1, ratio)
        end if
    end for

    return profile[n][2]
end function

-- Locate the SOFAR deep sound channel axis depth (depth of minimum sound speed)
public function svp_find_channel_axis(sequence profile)
    integer n, min_idx
    atom min_c

    n = length(profile)
    if n = 0 then
        return 0.0
    end if

    min_c = profile[1][2]
    min_idx = 1

    for i = 2 to n do
        if profile[i][2] < min_c then
            min_c = profile[i][2]
            min_idx = i
        end if
    end for

    return profile[min_idx][1]
end function
