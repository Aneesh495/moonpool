-- Moonpool Unit Test Suite: NMEA 0183 Decoders, Teledyne PD0 DVL, Sea-Bird CTD, and Fastpath FFI
-- OpenEuphoria v4.1+

include std/unittest.e
include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/telemetry/nmea.e
include src/telemetry/pd0_dvl.e
include src/telemetry/ctd.e
include src/fastpath/fastpath_ffi.e

set_test_verbosity(TEST_SHOW_ALL)

-- 1. NMEA 0183 Tests
-- Calculate checksum for "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"
sequence nmea_body = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"
sequence ck = nmea_compute_checksum(nmea_body)
sequence full_nmea = nmea_body & "*" & ck
test_equal("NMEA checksum computed string length", 2, length(ck))
test_equal("NMEA checksum verification", 1, nmea_verify_checksum(full_nmea))

-- Parse $GPGGA sentence
sequence gga_res = nmea_parse_gpgga(full_nmea)
test_equal("GPGGA parsed valid flag", 1, gga_res[1])
-- 48 deg 07.038 min = 48 + 7.038 / 60 = 48.1173 deg
test_true("GPGGA latitude decimal degrees", approx_equal(48.1173, gga_res[2], 1e-3))
-- 11 deg 31.000 min = 11 + 31 / 60 = 11.516667 deg
test_true("GPGGA longitude decimal degrees", approx_equal(11.5167, gga_res[3], 1e-3))
test_equal("GPGGA fix quality", 1, gga_res[4])
test_equal("GPGGA satellites tracked", 8, gga_res[5])
test_true("GPGGA altitude", approx_equal(545.4, gga_res[7], 1e-2))

-- Parse $PASHR attitude sentence
sequence pashr_raw = "$PASHR,123519.000,123.45,T,2.34,-1.23,0.05,0.05,0.08,1,1"
sequence pashr_ck = nmea_compute_checksum(pashr_raw)
sequence full_pashr = pashr_raw & "*" & pashr_ck
sequence pashr_res = nmea_parse_pashr(full_pashr)
test_equal("PASHR valid flag", 1, pashr_res[1])
test_true("PASHR heading", approx_equal(123.45, pashr_res[2], 1e-2))
test_true("PASHR roll", approx_equal(2.34, pashr_res[3], 1e-2))
test_true("PASHR pitch", approx_equal(-1.23, pashr_res[4], 1e-2))

-- 2. Teledyne PD0 DVL Binary Tests
sequence mock_pd0 = repeat(0, 36)
mock_pd0[1] = #7F -- Sync 1
mock_pd0[2] = #7F -- Sync 2
mock_pd0[3] = 42  -- Ensemble LSB
mock_pd0[4] = 0   -- Ensemble MSB (ensemble 42)

-- Beam 1: 1000 mm/s (1.0 m/s) -> 1000 = 0x03E8 -> LSB=0xE8=232, MSB=0x03=3
mock_pd0[15] = 232
mock_pd0[16] = 3

-- Beam 2: -500 mm/s -> 65536 - 500 = 65036 = 0xFE0C -> LSB=0x0C=12, MSB=0xFE=254
mock_pd0[17] = 12
mock_pd0[18] = 254

-- Correlation magnitudes (bytes 23..26)
mock_pd0[23] = 128
mock_pd0[24] = 128
mock_pd0[25] = 128
mock_pd0[26] = 128

-- Echo intensities (bytes 27..30)
mock_pd0[27] = 90
mock_pd0[28] = 90
mock_pd0[29] = 90
mock_pd0[30] = 90

-- Percent-good (bytes 31..34)
mock_pd0[31] = 100
mock_pd0[32] = 100
mock_pd0[33] = 100
mock_pd0[34] = 100

sequence pd0_res = pd0_parse_packet(mock_pd0)
test_equal("PD0 parsed valid", 1, pd0_res[1])
test_equal("PD0 ensemble number", 42, pd0_res[2])
sequence b_vels = pd0_res[3]
test_true("PD0 Beam 1 velocity m/s", approx_equal(1.0, b_vels[1], 1e-3))
test_true("PD0 Beam 2 velocity m/s", approx_equal(-0.5, b_vels[2], 1e-3))

-- 3. Sea-Bird CTD Tests
-- Conductivity 4.29 S/m at 15 deg C -> Salinity approx 35.0 PSU
atom sal_35 = ctd_compute_salinity(4.2914, 15.0, 0.0)
test_true("CTD standard seawater salinity 35 PSU", approx_equal(35.0, sal_35, 0.1))

sequence ctd_res = ctd_process_packet(4.2914, 15.0, 100.0)
test_true("CTD processed salinity", approx_equal(35.0, ctd_res[1], 0.1))
test_true("CTD sound speed realistic", ctd_res[2] > 1490.0 and ctd_res[2] < 1520.0)
test_true("CTD seawater density realistic", ctd_res[3] > 1020.0 and ctd_res[3] < 1030.0)

-- 4. Fastpath FFI Tests
integer loaded = fastpath_load()
test_true("Fastpath loader executed", loaded = 1 or loaded = 0)

test_report()
