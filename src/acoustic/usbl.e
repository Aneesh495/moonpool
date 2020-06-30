-- Moonpool Subsea Robotics Acoustic Navigation
-- Ultra-Short Baseline (USBL) Phase Interferometry and Slant Range Tracking
-- OpenEuphoria v4.1+ Specification

namespace moonpool_usbl

include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e
include src/core/quaternion.e

-- Calculate Direction of Arrival (DoA) unit vector from hydrophone phase differences
-- phase_diff_x, phase_diff_y: measured acoustic phase differences (radians)
-- d_x, d_y: hydrophone baseline spacing (meters)
-- wavelength_m: acoustic wavelength c / f (meters)
-- Returns body-frame unit vector u_b = {u_x, u_y, u_z} pointing towards subsea beacon
public function usbl_doa_from_phase(atom phase_diff_x, atom phase_diff_y, atom d_x, atom d_y, atom wavelength_m)
    atom k = TWO_PI / wavelength_m
    atom u_x, u_y, u_z, horiz_sq

    -- Direct phase to angle projection
    u_x = clamp(phase_diff_x / (k * d_x), -1.0, 1.0)
    u_y = clamp(phase_diff_y / (k * d_y), -1.0, 1.0)

    horiz_sq = u_x * u_x + u_y * u_y
    if horiz_sq >= 1.0 then
        -- Acoustic horizon limit
        atom scale_factor = 0.999 / sqrt(horiz_sq)
        u_x *= scale_factor
        u_y *= scale_factor
        u_z = sqrt(1.0 - (u_x * u_x + u_y * u_y))
    else
        u_z = sqrt(1.0 - horiz_sq)
    end if

    return {u_x, u_y, u_z}
end function

-- Calculate 3D subsea target position in local NED coordinates from USBL acoustic fix
-- vessel_pos_ned: {north, east, down} position of vessel reference point (meters)
-- vessel_att_quat: {w, x, y, z} vessel attitude orientation quaternion
-- lever_arm: {lx, ly, lz} transceiver mounting offset in vessel body coordinates (meters)
-- phase_x, phase_y: phase difference measurements (radians)
-- twtt_sec: Two-Way Travel Time (seconds)
-- sound_speed: average sound speed along acoustic path (m/s)
-- d_x, d_y: baseline array dimensions (meters)
-- freq_hz: acoustic carrier frequency (Hz)
-- Returns sequence {target_ned, slant_range_m, doa_ned}
public function usbl_solve_position(
    sequence vessel_pos_ned,
    sequence vessel_att_quat,
    sequence lever_arm,
    atom phase_x,
    atom phase_y,
    atom twtt_sec,
    atom sound_speed,
    atom d_x,
    atom d_y,
    atom freq_hz
)
    atom wavelength_m, slant_range_m
    sequence u_b, u_n, lever_ned, transceiver_ned, target_ned
    sequence R_b_n

    -- 1. Slant range from two-way travel time
    slant_range_m = 0.5 * twtt_sec * sound_speed
    wavelength_m = sound_speed / freq_hz

    -- 2. Direction of Arrival unit vector in transceiver body frame
    u_b = usbl_doa_from_phase(phase_x, phase_y, d_x, d_y, wavelength_m)

    -- 3. Vessel attitude rotation matrix
    R_b_n = quat_to_dcm(vessel_att_quat)

    -- 4. Rotate DoA vector and lever arm into NED navigation frame
    u_n = mat_vec_mul(R_b_n, u_b)
    lever_ned = mat_vec_mul(R_b_n, lever_arm)

    -- 5. Transceiver location in NED frame
    transceiver_ned = vec_add(vessel_pos_ned, lever_ned)

    -- 6. Beacon target location in NED frame
    target_ned = vec_add(transceiver_ned, vec_scale(u_n, slant_range_m))

    return {target_ned, slant_range_m, u_n}
end function

-- Estimate USBL positioning error covariance (3x3 matrix in NED frame)
-- range_std: standard deviation of range measurement (meters)
-- phase_std: standard deviation of phase measurement (radians)
public function usbl_covariance(atom slant_range_m, atom phase_std, atom range_std, atom sound_speed, atom freq_hz, atom baseline_m)
    atom wavelength = sound_speed / freq_hz
    atom angular_std = (phase_std * wavelength) / (TWO_PI * baseline_m)
    atom cross_range_std = slant_range_m * angular_std
    sequence cov

    cov = mat_zeros(3, 3)
    cov[1][1] = cross_range_std * cross_range_std
    cov[2][2] = cross_range_std * cross_range_std
    cov[3][3] = range_std * range_std

    return cov
end function
