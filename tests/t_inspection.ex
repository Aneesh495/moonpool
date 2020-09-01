-- Moonpool Unit Test Suite: Cathodic Protection, Anode Depletion, Ultrasonic NDT, and ASME B31G
-- OpenEuphoria v4.1+

include std/unittest.e
include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/inspection/cathodic.e
include src/inspection/anode.e
include src/inspection/ultrasonic.e
include src/inspection/b31g.e

set_test_verbosity(TEST_SHOW_ALL)

-- 1. Cathodic Protection Tests
sequence cp_adq = cp_evaluate_potential(-950.0)
test_equal("CP adequate status", CP_STATUS_ADEQUATE, cp_adq[1])

sequence cp_under = cp_evaluate_potential(-720.0)
test_equal("CP underprotected status", CP_STATUS_UNDERPROTECTED, cp_under[1])

sequence cp_over = cp_evaluate_potential(-1180.0)
test_equal("CP overprotected status", CP_STATUS_OVERPROTECTED, cp_over[1])

-- Electric field gradient: 10 mV drop over 1.0 meter = 100 uV/cm
atom grad = cp_field_gradient(-950.0, -940.0, 1.0)
test_true("CP electric field gradient calculation", approx_equal(100.0, grad, 1e-3))

-- Coating holiday detection
integer hol_detected = cp_detect_coating_holiday(-820.0, 120.0, 50.0)
test_equal("Coating holiday detected", 1, hol_detected)

-- 2. Sacrificial Anode Tests
-- McCoy resistance for 0.5 m^2 bracelet in 0.30 ohm*m seawater
-- R = 0.315 * 0.30 / sqrt(0.5) ~ 0.1336 ohms
atom r_mccoy = anode_resistance_mccoy(0.30, 0.50)
test_true("Anode McCoy resistance", approx_equal(0.1336, r_mccoy, 1e-2))

-- Delivered current with driving voltage from -1050 mV anode to -900 mV pipe = 150 mV = 0.15 V
atom i_out = anode_current_output(-1050.0, -900.0, 0.1336)
test_true("Anode current output", approx_equal(1.122, i_out, 1e-2))

-- Remaining life: 50 kg Al anode (2000 A*hr/kg) at 1.12 A current
-- Annual consumption = 1.12 * 8760 / 2000 = 4.905 kg/year
-- Usable mass = 50 * 0.80 = 40 kg -> ~8.15 years
atom life_yrs = anode_remaining_life_years(50.0, 50.0, 1.12, ANODE_CAPACITY_ALUMINUM, ANODE_UTILIZATION_FACTOR)
test_true("Anode remaining life estimation", life_yrs > 7.5 and life_yrs < 9.0)

-- 3. Ultrasonic NDT Tests
-- Longitudinal wave in steel: 5900 m/s
-- For 19.1 mm wall thickness, TWTT = 2 * 0.0191 / 5900 ~ 6.474e-6 seconds (6.474 us)
atom twtt_sample = (2.0 * 0.0191) / 5900.0
atom wall_t = ut_wall_thickness_mm(twtt_sample, 5900.0)
test_true("UT wall thickness calculation", approx_equal(19.1, wall_t, 1e-2))

-- Corrosion pit depth: nominal 20mm, measured 16mm -> 4mm pit (20% wall loss)
sequence corr_info = ut_analyze_corrosion(20.0, 16.0)
test_equal("UT pit depth", 4.0, corr_info[1])
test_equal("UT loss percentage", 20.0, corr_info[2])
test_equal("UT moderate pitting severity", NDT_DEFECT_MODERATE_PITTING, corr_info[3])

-- Multi-channel circumferential ring scan
sequence twtt_ring = {
    twtt_sample, twtt_sample, twtt_sample,
    (2.0 * 0.012) / 5900.0, -- Critical thinning at 6 o'clock (12mm wall)
    twtt_sample, twtt_sample
}
sequence scan_res = ut_scan_circumference(twtt_ring, 19.1, 5900.0)
test_true("Scan min thickness", approx_equal(12.0, scan_res[1], 1e-2))
test_equal("Scan critical sensor index", 4, scan_res[3])

-- 4. ASME B31G Burst Pressure Tests
-- 24-inch (609.6 mm) pipeline, 19.1 mm wall, API 5L X65 (448 MPa), Design Factor 0.72
-- Nominal MAOP = (2 * 448 * 19.1 / 609.6) * 0.72 * 10 = 201.8 bar
atom D_pipe = 609.6
atom t_pipe = 19.1
atom smys = 448.0
atom f_des = 0.72

-- Test shallow pit: 1.0 mm deep (5.2% depth), 50 mm length -> Acceptable at full MAOP
sequence b31g_ok = b31g_original_eval(D_pipe, t_pipe, smys, f_des, 1.0, 50.0)
test_equal("B31G light defect acceptable", B31G_STATUS_ACCEPTABLE, b31g_ok[4])
test_true("B31G FPR = 1.0", approx_equal(1.0, b31g_ok[3], 1e-3))

-- Test severe defect: 10.0 mm deep (52.3% depth), 200 mm length -> Requires derating
sequence b31g_severe = b31g_original_eval(D_pipe, t_pipe, smys, f_des, 10.0, 200.0)
test_true("B31G severe defect derated", b31g_severe[4] >= B31G_STATUS_DERATE)
test_true("B31G derated pressure < MAOP", b31g_severe[1] < b31g_severe[2])

-- Modified B31G evaluation
sequence b31g_mod = b31g_modified_eval(D_pipe, t_pipe, smys, f_des, 10.0, 200.0)
test_true("Modified B31G FPR valid", b31g_mod[3] > 0.0 and b31g_mod[3] < 1.0)

test_report()
