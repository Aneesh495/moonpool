-- Moonpool Subsea Robotics Telemetry Codecs
-- NMEA 0183 Protocol Parser, Checksum Validator, and Sentence Formatter
-- OpenEuphoria v4.1+ Specification

namespace moonpool_nmea

include std/math.e
include std/text.e
include std/sequence.e
include std/convert.e
include src/core/constants.e
include src/core/math_util.e

constant HEX_DIGITS = "0123456789ABCDEF"

-- Calculate 8-bit XOR checksum of NMEA string (characters between '$' and '*')
-- Returns 2-character uppercase hexadecimal string
public function nmea_compute_checksum(sequence sentence)
    integer xor_sum = 0
    integer start_idx = 1
    integer end_idx = length(sentence)

    if length(sentence) > 0 and sentence[1] = '$' then
        start_idx = 2
    end if

    for i = start_idx to end_idx do
        if sentence[i] = '*' then
            exit
        end if
        xor_sum = xor_bits(xor_sum, sentence[i])
    end for

    integer hi = floor(xor_sum / 16) + 1
    integer lo = remainder(xor_sum, 16) + 1
    if hi < 1 then hi = 1 end if
    if hi > 16 then hi = 16 end if
    if lo < 1 then lo = 1 end if
    if lo > 16 then lo = 16 end if

    return {HEX_DIGITS[hi], HEX_DIGITS[lo]}
end function

-- Verify if an NMEA sentence contains a valid matching checksum
public function nmea_verify_checksum(sequence sentence)
    integer star_pos = find('*', sentence)
    if star_pos = 0 or star_pos + 2 > length(sentence) then
        return 0 -- Missing checksum field
    end if

    sequence actual_ck = sentence[star_pos+1 .. star_pos+2]
    sequence expected_ck = nmea_compute_checksum(sentence[1 .. star_pos-1])

    return equal(upper(actual_ck), upper(expected_ck))
end function

-- Parse NMEA format latitude or longitude (DDMM.MMMM or DDDMM.MMMM) into decimal degrees
-- coord_str: string e.g. "5630.0000" (56 deg 30 min)
-- hemi_char: 'N', 'S', 'E', or 'W'
public function nmea_parse_latlon(sequence coord_str, integer hemi_char)
    atom deg_val, min_val, decimal_deg
    integer dot_pos

    if length(coord_str) < 4 then
        return 0.0
    end if

    dot_pos = find('.', coord_str)
    if dot_pos = 0 then
        dot_pos = length(coord_str) + 1
    end if

    integer deg_len = dot_pos - 3
    if deg_len <= 0 then
        return 0.0
    end if

    deg_val = to_number(coord_str[1 .. deg_len])
    min_val = to_number(coord_str[deg_len+1 .. length(coord_str)])

    decimal_deg = deg_val + (min_val / 60.0)

    if hemi_char = 'S' or hemi_char = 'W' then
        decimal_deg = -decimal_deg
    end if

    return decimal_deg
end function

-- Parse $GPGGA Sentence:
-- Format: $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
-- Returns sequence {valid, lat_deg, lon_deg, fix_quality, num_sats, hdop, alt_m}
public function nmea_parse_gpgga(sequence sentence)
    sequence tokens = split(sentence, ',')
    if length(tokens) < 10 or not nmea_verify_checksum(sentence) then
        return {0, 0.0, 0.0, 0, 0, 99.0, 0.0}
    end if

    sequence tag = tokens[1]
    if not equal(tag, "$GPGGA") and not equal(tag, "$INGGA") then
        return {0, 0.0, 0.0, 0, 0, 99.0, 0.0}
    end if

    sequence lat_str = tokens[3]
    sequence lat_hemi = tokens[4]
    sequence lon_str = tokens[5]
    sequence lon_hemi = tokens[6]
    sequence qual_str = tokens[7]
    sequence sats_str = tokens[8]
    sequence hdop_str = tokens[9]
    sequence alt_str  = tokens[10]

    atom lat_deg = 0.0
    atom lon_deg = 0.0
    if length(lat_hemi) > 0 and length(lat_str) > 0 then
        lat_deg = nmea_parse_latlon(lat_str, lat_hemi[1])
    end if
    if length(lon_hemi) > 0 and length(lon_str) > 0 then
        lon_deg = nmea_parse_latlon(lon_str, lon_hemi[1])
    end if

    integer fix_qual = to_integer(qual_str)
    integer num_sats = to_integer(sats_str)
    atom hdop_val = to_number(hdop_str)
    atom alt_val = to_number(alt_str)

    return {1, lat_deg, lon_deg, fix_qual, num_sats, hdop_val, alt_val}
end function

-- Parse $PASHR Attitude Sentence (Pitch, Roll, Heading):
-- Format: $PASHR,hhmmss.sss,hhh.hh,T,rrr.rr,ppp.pp,xxx.xx,...*hh
-- Returns sequence {valid, heading_deg, roll_deg, pitch_deg}
public function nmea_parse_pashr(sequence sentence)
    sequence tokens = split(sentence, ',')
    if length(tokens) < 6 or not nmea_verify_checksum(sentence) then
        return {0, 0.0, 0.0, 0.0}
    end if

    if not equal(tokens[1], "$PASHR") then
        return {0, 0.0, 0.0, 0.0}
    end if

    atom heading_deg = to_number(tokens[3])
    atom roll_deg    = to_number(tokens[5])
    atom pitch_deg   = to_number(tokens[6])

    return {1, heading_deg, roll_deg, pitch_deg}
end function
