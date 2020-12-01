-- Moonpool Unit Test Suite: Pipeline GIS Route, Coating Breakdown, Rainflow Fatigue, and Free Spans
-- OpenEuphoria v4.1+

include std/unittest.e
include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/pipeline/pipe_geometry.e
include src/pipeline/coating.e
include src/pipeline/fatigue.e
include src/pipeline/spans.e

set_test_verbosity(TEST_SHOW_ALL)

-- 1. Pipeline Geometry Tests
-- 24-inch (0.6096m) outer diameter, 19.1mm wall, 3.5mm 3LPE, 50mm concrete weight coating
sequence spec = pipe_spec_create(0.6096, 0.0191, 0.0035, 0.050, 7850.0, 2400.0)
atom sub_wt = pipe_submerged_weight(spec, 1025.0)
test_true("Pipeline submerged weight positive (sits stably)", sub_wt > 500.0)

-- Route interpolation along KP stations
sequence route = {
    {0.0,    0.0,   50.0, 100.0},
    {500.0,  500.0, 50.0, 102.0},
    {1000.0, 1000.0,50.0, 105.0}
}
sequence pos_kp250 = pipe_route_kp_to_pos(route, 250.0)
test_true("Route KP 250m North position", approx_equal(250.0, pos_kp250[1], 1e-2))
test_true("Route KP 250m East position", approx_equal(50.0, pos_kp250[2], 1e-2))
test_true("Route KP 250m Depth position", approx_equal(101.0, pos_kp250[3], 1e-2))

-- 2. Coating Degradation Tests
atom fb_init = coating_breakdown_factor(COATING_TYPE_3LPE, 0.0)
atom fb_25yr = coating_breakdown_factor(COATING_TYPE_3LPE, 25.0)
test_true("Coating breakdown factor initial small", fb_init < 0.02)
test_true("Coating breakdown factor increases over 25 years", fb_25yr > fb_init)

atom disbond_r = coating_disbondment_radius_mm(5.0, -1150.0, 15.0, 365.0)
test_true("Cathodic disbondment radius expands after 1 year", disbond_r > 5.0)

-- 3. Rainflow Fatigue Tests
-- S-N curve D: stress range 100 MPa -> cycles to failure
atom N_fail = sn_cycles_to_failure(100.0)
test_true("S-N cycles to failure realistic", N_fail > 1.0e6 and N_fail < 1.0e8)

-- Palmgren-Miner cumulative damage sum: 10,000 cycles at 100 MPa
atom D_damage = fatigue_miner_sum({100.0}, {10000.0})
test_true("Palmgren-Miner damage index positive and safe", D_damage > 0.0 and D_damage < 0.5)

-- Rainflow counting on simple cycle {0, 100, 0}
sequence rf_ranges = rainflow_count_ranges({0.0, 100.0, 0.0, 100.0})
test_true("Rainflow counted stress range", length(rf_ranges) > 0)

-- 4. Free Span Dynamics Tests
atom I_pipe = pipe_area_moment_inertia(0.6096, 0.0191)
test_true("Area moment of inertia positive", I_pipe > 0.0)

-- Span natural frequency for 30m span length
atom f0 = span_natural_frequency_hz(30.0, 0.6096, 0.0191, 450.0, 22.37)
test_true("Span natural frequency in realistic subsea range (0.5 to 5.0 Hz)", f0 > 0.5 and f0 < 5.0)

-- Vortex shedding at 1.0 m/s current: f_v = 0.20 * 1.0 / 0.6096 ~ 0.328 Hz
atom f_v = span_vortex_shedding_frequency_hz(1.0, 0.6096)
test_true("Vortex shedding frequency calculation", approx_equal(0.328, f_v, 1e-2))

-- VIV lock-in assessment
sequence viv_status = span_assess_viv_lockin(1.0, f0, 0.6096)
test_equal("VIV lock-in flag valid", 1, viv_status[1] = 0 or viv_status[1] = 1)

test_report()
