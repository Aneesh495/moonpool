-- Moonpool Subsea Robotics Acoustic Navigation
-- Phased Array Acoustic Beamforming, Spatial Array Manifolds, and Side-Lobe Suppression
-- OpenEuphoria v4.1+ Specification

namespace moonpool_beamforming

include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e

-- Construct steering vector / array manifold vector for hydrophone array:
-- hydrophone_pos: sequence of {x, y, z} coordinates for M hydrophone elements (meters)
-- doa_az_rad: target steering azimuth angle (radians)
-- doa_el_rad: target steering elevation angle (radians)
-- freq_hz: acoustic carrier frequency (Hz)
-- sound_speed: seawater sound speed (m/s)
-- Returns sequence of complex phase factors { {re1, im1}, {re2, im2}, ... }
public function beamforming_steering_vector(
    sequence hydrophone_pos,
    atom doa_az_rad,
    atom doa_el_rad,
    atom freq_hz,
    atom sound_speed
)
    integer m = length(hydrophone_pos)
    atom k = TWO_PI * freq_hz / sound_speed
    sequence u_steer, steer_vec

    -- Unit pointing direction vector in array coordinate frame:
    -- u = {cos(el) * cos(az), cos(el) * sin(az), sin(el)}
    u_steer = {
        cos(doa_el_rad) * cos(doa_az_rad),
        cos(doa_el_rad) * sin(doa_az_rad),
        sin(doa_el_rad)
    }

    steer_vec = repeat({0.0, 0.0}, m)
    for i = 1 to m do
        sequence pos = hydrophone_pos[i]
        -- Phase delay: phi_i = k * (pos_i . u)
        atom phase_delay = k * vec_dot(pos, u_steer)
        -- Complex exponential: exp(-j * phi) = {cos(-phi), sin(-phi)}
        steer_vec[i] = {cos(-phase_delay), sin(-phase_delay)}
    end for

    return steer_vec
end function

-- Conventional Delay-and-Sum (DAS) acoustic beamformer:
-- signals: sequence of M hydrophone time series samples at current instant
-- steer_vec: array manifold steering vector from beamforming_steering_vector
-- Returns combined scalar acoustic beam output amplitude
public function beamforming_delay_and_sum(sequence signals, sequence steer_vec)
    integer m = length(signals)
    atom re_sum = 0.0
    atom im_sum = 0.0

    for i = 1 to m do
        atom s = signals[i]
        sequence w = steer_vec[i]
        re_sum += s * w[1]
        im_sum += s * w[2]
    end for

    -- Normalization by number of array elements
    re_sum /= m
    im_sum /= m

    return sqrt(re_sum * re_sum + im_sum * im_sum)
end function

-- Compute directional acoustic power response across angular scan grid:
-- hydrophone_pos: array element layout
-- signals: time-slice measurements across array
-- scan_angles_deg: sequence of test angles {-60, -50, ..., +60}
public function beamforming_scan_1d(
    sequence hydrophone_pos,
    sequence signals,
    sequence scan_angles_deg,
    atom freq_hz,
    atom sound_speed
)
    integer n_angles = length(scan_angles_deg)
    sequence power_db = repeat(0.0, n_angles)

    for i = 1 to n_angles do
        atom az_rad = scan_angles_deg[i] * DEG2RAD
        sequence w = beamforming_steering_vector(hydrophone_pos, az_rad, 0.0, freq_hz, sound_speed)
        atom amp = beamforming_delay_and_sum(signals, w)
        if amp < 1.0e-6 then
            amp = 1.0e-6
        end if
        power_db[i] = 20.0 * log10(amp)
    end for

    return power_db
end function
