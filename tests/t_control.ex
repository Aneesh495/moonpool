-- Moonpool Unit Test Suite: 6-DOF Hydrodynamics, Thruster Allocation, PID, and LOS Guidance
-- OpenEuphoria v4.1+

include std/unittest.e
include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e
include src/control/hydrodynamics.e
include src/control/thruster_model.e
include src/control/allocation.e
include src/control/pid_controller.e
include src/control/guidance.e

set_test_verbosity(TEST_SHOW_ALL)

-- 1. Hydrodynamics Tests
-- Vehicle: 250 kg mass, 0.245 m^3 volume (approx neutrally buoyant in 1025 kg/m^3 water)
sequence veh = hydro_vehicle_create(
    250.0,
    0.245,
    {1.5, 0.8, 0.6},      -- {L, W, H} meters
    {0.0, 0.0, 0.05},     -- r_G center of gravity 5cm below geometric origin
    {0.0, 0.0, -0.05}     -- r_B center of buoyancy 5cm above geometric origin
)
sequence M_RB = hydro_rigid_body_mass(veh)
test_equal("Rigid body mass rows", 6, length(M_RB))
test_equal("Rigid body mass X", 250.0, M_RB[1][1])
test_equal("Rigid body mass Y", 250.0, M_RB[2][2])
test_equal("Rigid body mass Z", 250.0, M_RB[3][3])

sequence M_A = hydro_added_mass(veh, 1025.0)
test_equal("Added mass rows", 6, length(M_A))
test_true("Added mass X_udot positive", M_A[1][1] > 0.0)

-- Metacentric restoring moment: when pitched by 10 deg, restoring pitch moment is negative (restoring)
sequence g_rest = hydro_restoring_wrench(veh, 0.0, 10.0 * DEG2RAD, 1025.0)
test_true("Restoring pitch moment acts against pitch", g_rest[5] < 0.0)

-- 2. Thruster Model Tests
sequence thruster = thruster_create(350.0, -250.0, 500.0, 5.0)
-- Command within deadband
atom f_deadband = thruster_step_force(thruster, 3.0, 0.0, 0.1)
test_equal("Thruster deadband clamped to 0", 0.0, f_deadband)

-- Command forward force step
atom f_step = thruster_step_force(thruster, 100.0, 0.0, 0.1) -- Slew rate max = 500 * 0.1 = 50 N
test_true("Thruster slew rate ramped", approx_equal(50.0, f_step, 1e-3))

-- Saturated forward force
atom f_sat = thruster_step_force(thruster, 500.0, 340.0, 0.1)
test_equal("Thruster forward saturated", 350.0, f_sat)

-- 3. 8-Thruster Allocation Matrix Tests
sequence B_alloc = allocation_matrix_create(0.6, 0.4, 0.5, 0.3)
test_equal("Allocation matrix rows", 6, length(B_alloc))
test_equal("Allocation matrix cols", 8, length(B_alloc[1]))

-- Commanded pure surge force tau = {100.0 N, 0, 0, 0, 0, 0}
sequence u_surge = allocation_solve(
    {100.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    B_alloc,
    repeat(1.0, 8), -- All 8 thrusters healthy
    0.001
)
test_equal("Allocated forces count", 8, length(u_surge))
-- Symmetric forward thrust: T1 and T2 should push forward (+), T3 and T4 push reverse (-)
test_true("T1 thrust positive", u_surge[1] > 0.0)
test_true("T2 thrust positive", u_surge[2] > 0.0)
test_true("Vertical thrusters zero during pure surge", approx_equal(0.0, u_surge[5], 1e-3))

-- Fault tolerance test: thruster 2 fails (health weight = 0.0)
sequence health = {1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}
sequence u_fault = allocation_solve(
    {100.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    B_alloc,
    health,
    0.001
)
test_equal("Failed thruster commanded 0 force", 0.0, u_fault[2])

-- 4. 6-DOF PID Controller Tests
sequence gains_6dof = {
    {50.0, 2.0, 10.0, 50.0, 200.0}, -- Surge
    {50.0, 2.0, 10.0, 50.0, 200.0}, -- Sway
    {80.0, 5.0, 20.0, 80.0, 300.0}, -- Heave
    {20.0, 1.0, 5.0,  20.0, 50.0},  -- Roll
    {30.0, 1.0, 8.0,  30.0, 80.0},  -- Pitch
    {40.0, 2.0, 15.0, 40.0, 100.0}  -- Yaw
}
sequence pid_ctrl = pid_controller_6dof_create(gains_6dof)
sequence setpoints = {10.0, 0.0, 50.0, 0.0, 0.0, 0.0}
sequence measurements = {0.0, 0.0, 50.0, 0.0, 0.0, 0.0}
sequence pid_res = pid_controller_6dof_step(pid_ctrl, setpoints, measurements, 0.1)
sequence tau_cmd = pid_res[1]

-- Surge error = 10m -> positive surge force commanded
test_true("PID surge output positive", tau_cmd[1] > 0.0)
test_true("PID heave output zero at setpoint", approx_equal(0.0, tau_cmd[3], 1e-3))

-- 5. Line-of-Sight Guidance Tests
-- Path straight East: Start {0, 0, 50m}, End {0, 100m, 50m}
-- Vehicle at {5.0, 20.0, 50.0} (5m North of path)
sequence los_res = los_guidance_step(
    {5.0, 20.0, 50.0},
    {0.0, 0.0, 50.0},
    {0.0, 100.0, 50.0},
    10.0, -- 10m lookahead
    3.0   -- 3m acceptance
)
atom psi_los = los_res[1]
atom e_cross = los_res[3]
test_equal("LOS cross-track error", -5.0, e_cross)
-- Path heading is East (+90 deg / PI/2). With vehicle North of path, steering angle turns South (> PI/2)
test_true("LOS steering turns toward path", psi_los > HALF_PI)

test_report()
