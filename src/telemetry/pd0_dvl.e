-- Moonpool Subsea Robotics Telemetry Codecs
-- Teledyne RDI Workhorse DVL Binary PD0/PD4 Telemetry Frame Decoder
-- OpenEuphoria v4.1+ Specification

namespace moonpool_pd0

include std/math.e
include std/sequence.e
include src/core/constants.e
include src/core/math_util.e

public constant PD0_HEADER_SYNC_BYTE = #7F
public constant PD0_BAD_VELOCITY_VAL = -32768

-- Decode a 16-bit signed integer (little-endian: lsb, msb)
public function decode_int16_le(integer lsb, integer msb)
    integer val = lsb + msb * 256
    if val >= 32768 then
        val -= 65536
    end if
    return val
end function

-- Decode an unsigned 16-bit integer (little-endian)
public function decode_uint16_le(integer lsb, integer msb)
    return lsb + msb * 256
end function

-- Parse synthetic or binary PD0 DVL bottom-track packet:
-- bytes: sequence of integer bytes (0 to 255)
-- Returns sequence {valid, ensemble_num, beam_velocities_m_s, correlations, intensities, percent_good}
public function pd0_parse_packet(sequence bytes)
    integer n_bytes = length(bytes)
    if n_bytes < 32 then
        return {0, 0, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}}
    end if

    -- Verify sync bytes
    if bytes[1] != PD0_HEADER_SYNC_BYTE or bytes[2] != PD0_HEADER_SYNC_BYTE then
        return {0, 0, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}}
    end if

    integer ensemble_num = decode_uint16_le(bytes[3], bytes[4])

    -- Extract 4 beam velocities (stored in mm/s starting at byte 15)
    sequence beam_vels = repeat(0.0, 4)
    for i = 1 to 4 do
        integer offset = 15 + (i - 1) * 2
        integer raw_v = decode_int16_le(bytes[offset], bytes[offset+1])
        if raw_v = PD0_BAD_VELOCITY_VAL then
            beam_vels[i] = 0.0 -- Flagged bad velocity
        else
            beam_vels[i] = raw_v / 1000.0 -- Convert mm/s to m/s
        end if
    end for

    -- Extract correlation magnitudes (bytes 23..26)
    sequence correlations = {bytes[23], bytes[24], bytes[25], bytes[26]}

    -- Extract echo intensities (bytes 27..30)
    sequence intensities = {bytes[27], bytes[28], bytes[29], bytes[30]}

    -- Extract percent-good statistics (bytes 31..34)
    sequence percent_good = {100, 100, 100, 100}
    if n_bytes >= 34 then
        percent_good = {bytes[31], bytes[32], bytes[33], bytes[34]}
    end if

    return {1, ensemble_num, beam_vels, correlations, intensities, percent_good}
end function

-- Check beam quality thresholds: returns 1 if beam satisfies correlation and percent-good criteria
public function pd0_is_beam_acceptable(atom vel_m_s, integer corr_mag, integer pct_good)
    if abs(vel_m_s) > 15.0 then
        return 0 -- Physical limit exceeded
    end if
    if corr_mag < 64 then
        return 0 -- Low correlation magnitude (poor acoustic SNR)
    end if
    if pct_good < 50 then
        return 0 -- Low 3-ping ensemble acceptance percentage
    end if
    return 1
end function
