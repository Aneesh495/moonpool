-- Moonpool Subsea Robotics Propulsion and Control
-- SNAME (1950) 6-DOF Marine Hydrodynamics, Added Mass, and Restoring Forces
-- OpenEuphoria v4.1+ Specification

namespace moonpool_hydrodynamics

include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e

-- Create vehicle hydrodynamic model parameter structure:
-- mass_kg: dry vehicle mass in air (kg)
-- volume_m3: displaced volume of hull (m^3)
-- dims: {length_m, width_m, height_m}
-- r_G: {xg, yg, zg} center of gravity relative to origin (meters)
-- r_B: {xb, yb, zb} center of buoyancy relative to origin (meters)
-- Returns vehicle structure
public function hydro_vehicle_create(
    atom mass_kg,
    atom volume_m3,
    sequence dims,
    sequence r_G,
    sequence r_B
)
    return {mass_kg, volume_m3, dims, r_G, r_B}
end function

-- Construct 6x6 rigid-body inertia matrix M_RB
public function hydro_rigid_body_mass(sequence veh)
    atom m = veh[1]
    sequence r_G = veh[4]
    atom xg = r_G[1], yg = r_G[2], zg = r_G[3]
    sequence dims = veh[3]
    atom L = dims[1], W = dims[2], H = dims[3]

    -- Principal moments of inertia for rectangular cuboid approximation
    atom Ixx = (1.0 / 12.0) * m * (W*W + H*H)
    atom Iyy = (1.0 / 12.0) * m * (L*L + H*H)
    atom Izz = (1.0 / 12.0) * m * (L*L + W*W)

    sequence M_RB = mat_zeros(6, 6)

    -- Translational mass
    M_RB[1][1] = m
    M_RB[2][2] = m
    M_RB[3][3] = m

    -- Rotational inertia
    M_RB[4][4] = Ixx
    M_RB[5][5] = Iyy
    M_RB[6][6] = Izz

    -- Off-diagonal coupling with center of gravity
    M_RB[1][5] =  m * zg
    M_RB[1][6] = -m * yg
    M_RB[2][4] = -m * zg
    M_RB[2][6] =  m * xg
    M_RB[3][4] =  m * yg
    M_RB[3][5] = -m * xg

    M_RB[5][1] =  m * zg
    M_RB[6][1] = -m * yg
    M_RB[4][2] = -m * zg
    M_RB[6][2] =  m * xg
    M_RB[4][3] =  m * yg
    M_RB[5][3] = -m * xg

    return M_RB
end function

-- Construct 6x6 hydrodynamic added mass matrix M_A
-- For prolate spheroid / box-shaped subsea ROV hull
public function hydro_added_mass(sequence veh, atom density_kg_m3)
    atom vol = veh[2]
    atom displaced_mass = density_kg_m3 * vol
    sequence dims = veh[3]
    atom L = dims[1], W = dims[2], H = dims[3]

    sequence M_A = mat_zeros(6, 6)

    -- Empirical added mass coefficients (DNV-RP-F105 / Fossen)
    M_A[1][1] = 0.20 * displaced_mass -- X_udot (Surge)
    M_A[2][2] = 0.85 * displaced_mass -- Y_vdot (Sway)
    M_A[3][3] = 0.95 * displaced_mass -- Z_wdot (Heave)

    -- Rotational added moments of inertia
    M_A[4][4] = 0.15 * displaced_mass * (W*W + H*H) -- K_pdot (Roll)
    M_A[5][5] = 0.60 * displaced_mass * (L*L + H*H) -- M_qdot (Pitch)
    M_A[6][6] = 0.60 * displaced_mass * (L*L + W*W) -- N_rdot (Yaw)

    return M_A
end function

-- Calculate hydrodynamic damping wrench D(nu) * nu (6-element vector)
-- nu: body velocity vector {u, v, w, p, q, r}
-- Combines linear skin friction and quadratic form drag
public function hydro_damping_wrench(sequence nu)
    sequence d_lin = {50.0, 120.0, 140.0, 15.0, 45.0, 45.0}
    sequence d_quad = {120.0, 240.0, 280.0, 30.0, 80.0, 80.0}
    sequence tau_d = repeat(0.0, 6)

    for i = 1 to 6 do
        atom v = nu[i]
        tau_d[i] = d_lin[i] * v + d_quad[i] * abs(v) * v
    end for

    return tau_d
end function

-- Calculate hydrostatic restoring forces and moments g(eta) (6-element vector)
-- roll_rad, pitch_rad: vehicle attitude angles (radians)
-- veh: vehicle parameter structure
-- density_kg_m3: seawater density (kg/m^3)
public function hydro_restoring_wrench(sequence veh, atom roll_rad, atom pitch_rad, atom density_kg_m3)
    atom m = veh[1]
    atom vol = veh[2]
    sequence r_G = veh[4]
    sequence r_B = veh[5]

    atom W = m * GRAVITY_STANDARD
    atom B = density_kg_m3 * vol * GRAVITY_STANDARD

    atom xg = r_G[1], yg = r_G[2], zg = r_G[3]
    atom xb = r_B[1], yb = r_B[2], zb = r_B[3]

    atom s_phi = sin(roll_rad)
    atom c_phi = cos(roll_rad)
    atom s_theta = sin(pitch_rad)
    atom c_theta = cos(pitch_rad)

    sequence g_vec = repeat(0.0, 6)

    -- Hydrostatic forces: (W - B) acting on Surge, Sway, Heave
    g_vec[1] =  (W - B) * s_theta
    g_vec[2] = -(W - B) * c_theta * s_phi
    g_vec[3] = -(W - B) * c_theta * c_phi

    -- Hydrostatic metacentric restoring moments: Roll, Pitch, Yaw
    g_vec[4] =  (yg * W - yb * B) * c_theta * c_phi + (zg * W - zb * B) * c_theta * s_phi
    g_vec[5] = -(zg * W - zb * B) * s_theta         - (xg * W - xb * B) * c_theta * c_phi
    g_vec[6] = -(xg * W - xb * B) * c_theta * s_phi - (yg * W - yb * B) * s_theta

    return g_vec
end function

-- Forward simulation of 6-DOF vehicle body accelerations:
-- nu_dot = (M_RB + M_A)^-1 * (tau_cmd - tau_damping - g_restoring)
public function hydro_forward_acceleration(
    sequence veh,
    sequence nu,
    sequence tau_cmd,
    atom roll_rad,
    atom pitch_rad,
    atom density_kg_m3
)
    sequence M_RB = hydro_rigid_body_mass(veh)
    sequence M_A = hydro_added_mass(veh, density_kg_m3)
    sequence M_total = mat_add(M_RB, M_A)
    sequence M_inv = mat_inv(M_total)

    sequence tau_d = hydro_damping_wrench(nu)
    sequence g_rest = hydro_restoring_wrench(veh, roll_rad, pitch_rad, density_kg_m3)

    sequence tau_net = repeat(0.0, 6)
    for i = 1 to 6 do
        tau_net[i] = tau_cmd[i] - tau_d[i] - g_rest[i]
    end for

    return mat_vec_mul(M_inv, tau_net)
end function
