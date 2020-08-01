-- Moonpool Subsea Robotics Bathymetry and Terrain Mapping
-- Multibeam Echo Sounder (MBES) Swath Projection and 3D Motion-Compensated Soundings
-- OpenEuphoria v4.1+ Specification

namespace moonpool_multibeam

include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e
include src/core/quaternion.e

-- Create a multibeam sonar configuration structure:
-- n_beams: number of receive beams (e.g. 128 or 256)
-- swath_angle_deg: total swath fan angle (e.g. 120.0 deg)
-- lever_arm: {lx, ly, lz} mounting offset in vehicle body frame (meters)
-- freq_hz: acoustic operating frequency (e.g. 400,000 Hz for high-res survey)
public function mbes_config_create(integer n_beams, atom swath_angle_deg, sequence lever_arm, atom freq_hz)
    return {n_beams, swath_angle_deg, lever_arm, freq_hz}
end function

-- Compute beam pointing angles relative to sonar boresight
-- Returns sequence of angles in radians, centered at 0 (nadir)
public function mbes_beam_angles(sequence config)
    integer n_beams = config[1]
    atom total_swath_rad = config[2] * DEG2RAD
    atom angle_step = total_swath_rad / (n_beams - 1)
    atom start_angle = -0.5 * total_swath_rad
    sequence angles = repeat(0.0, n_beams)

    for i = 1 to n_beams do
        angles[i] = start_angle + (i - 1) * angle_step
    end for

    return angles
end function

-- Project a raw multibeam ping swath into 3D georeferenced NED soundings
-- config: MBES configuration
-- veh_pos_ned: {north, east, down} vehicle navigation position (meters)
-- veh_att_quat: {w, x, y, z} vehicle attitude quaternion
-- travel_times: sequence of two-way travel times for each beam (seconds)
-- sound_speed: water column average sound speed (m/s)
-- Returns sequence of 3D sounding points {{n1, e1, d1}, {n2, e2, d2}, ...}
public function mbes_project_swath(
    sequence config,
    sequence veh_pos_ned,
    sequence veh_att_quat,
    sequence travel_times,
    atom sound_speed
)
    integer n_beams = config[1]
    sequence lever_arm = config[3]
    sequence beam_angles = mbes_beam_angles(config)
    sequence R_b_n = quat_to_dcm(veh_att_quat)
    sequence lever_ned = mat_vec_mul(R_b_n, lever_arm)
    sequence sonar_pos_ned = vec_add(veh_pos_ned, lever_ned)
    sequence soundings = {}

    for i = 1 to n_beams do
        atom twtt = travel_times[i]
        if twtt > 0.001 and twtt < 10.0 then
            atom r = 0.5 * twtt * sound_speed
            atom theta = beam_angles[i]

            -- Beam vector in sonar body frame:
            -- x = 0 (perpendicular swath)
            -- y = r * sin(theta) (athwartships / port-starboard)
            -- z = r * cos(theta) (downwards)
            sequence p_beam_body = {0.0, r * sin(theta), r * cos(theta)}

            -- Rotate beam vector into NED frame
            sequence p_beam_ned = mat_vec_mul(R_b_n, p_beam_body)

            -- Absolute sounding position
            sequence pt_ned = vec_add(sonar_pos_ned, p_beam_ned)
            soundings = append(soundings, pt_ned)
        end if
    end for

    return soundings
end function

-- Compute horizontal swath coverage footprint width across seabed (meters)
public function mbes_swath_width(atom altitude_m, atom swath_angle_deg)
    atom half_swath = 0.5 * swath_angle_deg * DEG2RAD
    return 2.0 * altitude_m * tan(half_swath)
end function
