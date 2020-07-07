-- Moonpool Unit Test Suite: Acoustic Navigation, SVP Modeling, Ray Tracing, USBL, and LBL
-- OpenEuphoria v4.1+

include std/unittest.e
include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e
include src/core/quaternion.e
include src/acoustic/sound_speed.e
include src/acoustic/ray_trace.e
include src/acoustic/usbl.e
include src/acoustic/lbl.e
include src/acoustic/multipath.e

set_test_verbosity(TEST_SHOW_ALL)

-- 1. Sound Speed Tests
-- At T=10 deg C, S=35 PSU, D=100m, Mackenzie gives ~1490 m/s
atom c_mack = sound_speed_mackenzie(10.0, 35.0, 100.0)
test_true("Mackenzie sound speed realistic range", c_mack > 1485.0 and c_mack < 1500.0)

atom c_med = sound_speed_medwin(10.0, 35.0, 100.0)
test_true("Medwin sound speed realistic range", c_med > 1485.0 and c_med < 1500.0)

-- SVP Interpolation
sequence svp_profile = {
    {0.0, 1505.0},
    {50.0, 1495.0},
    {200.0, 1482.0},
    {500.0, 1480.0},
    {1000.0, 1484.0}
}
atom c_interp = svp_interpolate(svp_profile, 25.0)
test_true("SVP interpolation midpoint", approx_equal(1500.0, c_interp, 1e-3))

atom axis_depth = svp_find_channel_axis(svp_profile)
test_equal("SOFAR channel axis depth", 500.0, axis_depth)

-- 2. Ray Tracing Tests
atom p = snell_parameter(0.0, 1500.0)
test_true("Snell parameter horizontal", approx_equal(1.0 / 1500.0, p, 1e-7))

-- Straight line layer test (iso-velocity)
sequence layer_res = trace_layer(p, 0.0, 50.0, 1500.0, 1500.0, 45.0 * DEG2RAD)
test_true("Iso-velocity layer delta depth", layer_res[1] > 0.0)
test_true("Iso-velocity layer delta time", layer_res[2] > 0.0)

-- Multi-layer ray trace
sequence ray_res = ray_trace_profile(svp_profile, 10.0, 200.0, 30.0)
test_true("Profile ray trace time positive", ray_res[1] > 0.0)
test_true("Profile ray trace range positive", ray_res[2] > 0.0)

-- 3. USBL Tests
atom baseline_d = 0.1 -- 10cm hydrophone spacing
atom freq = 25000.0   -- 25 kHz
atom c_avg = 1500.0
atom lambda_val = c_avg / freq -- 0.06m

-- Directly broadside (zero phase difference) -> DoA pointing straight down {0, 0, 1}
sequence u_b = usbl_doa_from_phase(0.0, 0.0, baseline_d, baseline_d, lambda_val)
test_true("USBL broadside DoA X", approx_equal(0.0, u_b[1], 1e-4))
test_true("USBL broadside DoA Y", approx_equal(0.0, u_b[2], 1e-4))
test_true("USBL broadside DoA Z", approx_equal(1.0, u_b[3], 1e-4))

-- USBL 3D positioning test
sequence vessel_pos = {100.0, 200.0, 0.0}
sequence vessel_att = quat_identity()
sequence lever_arm = {0.0, 0.0, 5.0} -- Transceiver 5m below vessel reference
atom twtt = 0.2 -- 0.2s two-way travel time -> 150m slant range
sequence usbl_sol = usbl_solve_position(
    vessel_pos, vessel_att, lever_arm,
    0.0, 0.0, twtt, c_avg,
    baseline_d, baseline_d, freq
)
sequence beacon_ned = usbl_sol[1]
test_true("USBL beacon North", approx_equal(100.0, beacon_ned[1], 1e-2))
test_true("USBL beacon East", approx_equal(200.0, beacon_ned[2], 1e-2))
test_true("USBL beacon Down", approx_equal(155.0, beacon_ned[3], 1e-2)) -- 0 + 5m + 150m = 155m

-- 4. LBL Multilateration Tests
-- 4 transponders placed on seabed at depth 500m
sequence transponders = {
    {0.0, 0.0, 500.0},
    {200.0, 0.0, 500.0},
    {0.0, 200.0, 500.0},
    {200.0, 200.0, 500.0}
}
-- True vehicle position: {100, 100, 450} (50m above transponder baseline)
sequence true_pos = {100.0, 100.0, 450.0}
sequence ranges = repeat(0.0, 4)
for i = 1 to 4 do
    ranges[i] = vec_norm(vec_sub(true_pos, transponders[i]))
end for

sequence init_guess = {90.0, 110.0, 440.0}
sequence lbl_sol = lbl_solve_position(transponders, ranges, init_guess, 1e-4, 20)
sequence est_pos = lbl_sol[1]
integer converged = lbl_sol[4]

test_equal("LBL Gauss-Newton convergence", 1, converged)
test_true("LBL solved North", approx_equal(100.0, est_pos[1], 1e-3))
test_true("LBL solved East", approx_equal(100.0, est_pos[2], 1e-3))
test_true("LBL solved Down", approx_equal(450.0, est_pos[3], 1e-3))

-- 5. Multipath Discrimination Tests
sequence arrivals = multipath_arrival_times(1000.0, 50.0, 400.0, 500.0, 1500.0)
test_true("Direct path earlier than surface bounce", arrivals[1] < arrivals[2])
test_true("Surface bounce earlier than seabed bounce", arrivals[2] < arrivals[3])

sequence class_direct = classify_acoustic_arrival(2.0 * arrivals[1], 1000.0, 50.0, 400.0, 500.0, 1500.0, 0.005)
test_equal("Direct arrival classified correctly", 0, class_direct[1])

sequence class_surface = classify_acoustic_arrival(2.0 * arrivals[2], 1000.0, 50.0, 400.0, 500.0, 1500.0, 0.005)
test_equal("Surface bounce detected as multipath", 1, class_surface[1])
test_equal("Surface bounce type tag", 1, class_surface[2])

test_report()
