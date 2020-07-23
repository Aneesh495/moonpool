-- Moonpool Unit Test Suite: Navigation, DVL Janus Processing, Pressure Depth, and 15-State ES-EKF
-- OpenEuphoria v4.1+

include std/unittest.e
include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e
include src/core/quaternion.e
include src/navigation/dvl.e
include src/navigation/depth_sensor.e
include src/navigation/dead_reckon.e
include src/navigation/ekf.e

set_test_verbosity(TEST_SHOW_ALL)

-- 1. DVL Tests
-- Pure forward motion test: equal and opposite beam velocities in symmetric Janus geometry
atom pitch_deg = 30.0
sequence M_janus = dvl_janus_matrix(pitch_deg)
test_equal("DVL Janus rows", 4, length(M_janus))
test_equal("DVL Janus cols", 4, length(M_janus[1]))

-- Simulate forward vehicle motion of 1.0 m/s surge:
-- b1 = +1.0 * sin(30) / sqrt(2)
-- b4 = +1.0 * sin(30) / sqrt(2)
-- b2 = -1.0 * sin(30) / sqrt(2)
-- b3 = -1.0 * sin(30) / sqrt(2)
atom b_forward = 1.0 * sin(30.0 * DEG2RAD) / sqrt(2.0)
sequence raw_beams = {b_forward, -b_forward, -b_forward, b_forward}
sequence v_body = dvl_beam_to_body(raw_beams, pitch_deg, 1500.0, 1500.0)

test_true("DVL surge velocity solved", approx_equal(1.0, v_body[1], 1e-3))
test_true("DVL sway velocity zero", approx_equal(0.0, v_body[2], 1e-3))
test_true("DVL heave velocity zero", approx_equal(0.0, v_body[3], 1e-3))
test_true("DVL error velocity zero", approx_equal(0.0, v_body[4], 1e-3))

-- DVL Altitude calculation
sequence slant_ranges = {10.0, 10.0, 10.0, 10.0} -- 10m at 30 deg -> 10 * cos(30 deg) = 8.66m
atom alt = dvl_calculate_altitude(slant_ranges, 30.0)
test_true("DVL altitude check", approx_equal(8.66025, alt, 1e-2))

-- 2. Depth Sensor Tests
atom g_equator = gravity_latitude(0.0)
atom g_pole = gravity_latitude(90.0)
test_true("Gravity latitude gradient", g_pole > g_equator)

-- Saunders-Fofonoff depth: 100 dbar is roughly 99.2m
atom d_saunders = pressure_to_depth_saunders(100.0, 56.5)
test_true("Saunders depth realistic range", d_saunders > 98.0 and d_saunders < 101.0)

-- 3. Dead Reckoning Tests
sequence init_pos = {0.0, 0.0, 100.0}
sequence init_vel = {1.0, 0.0, 0.0}
sequence init_q = quat_identity()
sequence dr_state = dr_state_create(init_pos, init_vel, init_q)

-- Propagate for 10 seconds at 1.0 m/s surge with zero gyro rate
for i = 1 to 10 do
    dr_state = dr_propagate(dr_state, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 1.0)
end for

sequence final_pos = dr_state[1]
test_true("DR final North position", approx_equal(10.0, final_pos[1], 1e-2))
test_true("DR final East position", approx_equal(0.0, final_pos[2], 1e-2))
test_true("DR final Down position", approx_equal(100.0, final_pos[3], 1e-2))

-- 4. 15-State ES-EKF Tests
sequence ekf = ekf_create(
    {0.0, 0.0, 100.0}, -- init pos
    {0.0, 0.0, 0.0},   -- init vel
    quat_identity(),   -- init quat
    1.0,               -- pos var
    0.1,               -- vel var
    0.01               -- att var
)

-- Predict with zero acceleration (only gravity along +z in body)
sequence accel_imu = {0.0, 0.0, -GRAVITY_STANDARD} -- Specific force counteracting gravity
sequence gyro_imu = {0.0, 0.0, 0.0}

ekf = ekf_predict(ekf, accel_imu, gyro_imu, 0.1)
sequence ekf_nom = ekf[1]
sequence ekf_pos = ekf_nom[1]
test_true("EKF stationary predict North", approx_equal(0.0, ekf_pos[1], 1e-2))
test_true("EKF stationary predict Down", approx_equal(100.0, ekf_pos[3], 1e-2))

-- Update with position measurement {10.0, 5.0, 102.0}
ekf = ekf_update_position(ekf, {10.0, 5.0, 102.0}, 0.5)
ekf_nom = ekf[1]
sequence updated_pos = ekf_nom[1]
-- Position should pull towards measurement
test_true("EKF position update pull North", updated_pos[1] > 2.0 and updated_pos[1] < 10.0)
test_true("EKF position update pull East", updated_pos[2] > 1.0 and updated_pos[2] < 5.0)

-- Update with DVL velocity {1.5, 0.0, 0.0}
ekf = ekf_update_dvl(ekf, {1.5, 0.0, 0.0}, 0.05)
ekf_nom = ekf[1]
sequence updated_vel = ekf_nom[2]
test_true("EKF DVL velocity update pull", updated_vel[1] > 0.5)

-- Update with depth sensor 101.5m
ekf = ekf_update_depth(ekf, 101.5, 0.01)
ekf_nom = ekf[1]
test_true("EKF depth update pull", approx_equal(101.5, ekf_nom[1][3], 0.2))

test_report()
