-- Moonpool Unit Test Suite: Oceanographic Physics, Tides, Waves, Currents, and Beamforming
-- OpenEuphoria v4.1+

include std/unittest.e
include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e
include src/ocean/tides.e
include src/ocean/waves.e
include src/ocean/currents.e
include src/acoustic/beamforming.e

set_test_verbosity(TEST_SHOW_ALL)

-- 1. Tidal Model Tests
sequence tide_model = {
    {TIDE_OMEGA_M2, 1.5, 30.0}, -- M2 tide (1.5m amplitude)
    {TIDE_OMEGA_S2, 0.5, 45.0}  -- S2 tide (0.5m amplitude)
}
atom eta_tide = tide_surface_elevation(tide_model, 0.0)
test_true("Tidal elevation within spring amplitude bounds", abs(eta_tide) <= 2.0)

atom v_tide = tide_stream_velocity(1.2, 3600.0, 0.0)
test_true("Tidal stream velocity within peak bounds", abs(v_tide) <= 1.2)

-- 2. Wave Theory Tests
atom k_wave = wave_solve_wavenumber(TWO_PI / 10.0, 100.0)
test_true("Wave dispersion wavenumber positive", k_wave > 0.03 and k_wave < 0.05)

-- Subsea orbital velocity attenuation at 50m depth vs 10m depth
sequence orb_10m = wave_orbital_velocities(3.0, 10.0, 10.0, 100.0, 0.0)
sequence orb_50m = wave_orbital_velocities(3.0, 10.0, 50.0, 100.0, 0.0)
test_true("Wave orbital velocity attenuates with depth", abs(orb_10m[1]) > abs(orb_50m[1]))

-- 3. Ekman Spiral Current Tests
sequence ekman = ekman_current_profile(15.0, 0.0, 56.5)
test_true("Ekman surface current magnitude realistic (0.2 to 0.6 m/s)", vec_norm(ekman) > 0.2 and vec_norm(ekman) < 0.6)

-- 4. Acoustic Beamforming Tests
-- 4-element linear array along X axis: broadside is normal to array (azimuth = 90 deg / HALF_PI)
sequence array_pos = {
    {-0.045, 0.0, 0.0},
    {-0.015, 0.0, 0.0},
    { 0.015, 0.0, 0.0},
    { 0.045, 0.0, 0.0}
}
sequence steer_broadside = beamforming_steering_vector(array_pos, HALF_PI, 0.0, 25000.0, 1500.0)
test_equal("Steering vector elements count", 4, length(steer_broadside))

sequence signals = {1.0, 1.0, 1.0, 1.0}
atom das_amp = beamforming_delay_and_sum(signals, steer_broadside)
test_true("Delay-and-Sum broadside coherent gain", approx_equal(1.0, das_amp, 1e-3))

test_report()
