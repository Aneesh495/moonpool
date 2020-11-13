-- Moonpool Subsea Robotics Pipeline Engineering
-- Pipeline Free Span Dynamics, Vortex-Induced Vibration (VIV), and Lock-In Resonance
-- OpenEuphoria v4.1+ Specification

namespace moonpool_spans

include std/math.e
include src/core/constants.e
include src/core/math_util.e

-- Steel Elasticity and Structural Parameters
public constant STEEL_YOUNGS_MODULUS_PA = 2.07e11 -- 207 GPa
public constant STROUHAL_NUMBER_SUBSEA   = 0.20    -- Standard subsea cylinder vortex shedding Strouhal number

-- Calculate second moment of area (area moment of inertia) for circular pipe cross-section (m^4):
-- I = (PI / 64) * (D_outer^4 - D_inner^4)
public function pipe_area_moment_inertia(atom D_outer_m, atom t_wall_m)
    atom D_inner = D_outer_m - 2.0 * t_wall_m
    atom d_out4 = D_outer_m * D_outer_m * D_outer_m * D_outer_m
    atom d_in4  = D_inner * D_inner * D_inner * D_inner
    return (PI / 64.0) * (d_out4 - d_in4)
end function

-- Calculate fundamental natural frequency of pipeline free span (Hz):
-- span_len_m: length of unsupported pipe span (meters)
-- D_outer_m: outer diameter (meters)
-- t_wall_m: wall thickness (meters)
-- m_eff_kg_m: effective mass per unit length (pipe + internal gas/oil + external added mass)
-- boundary_coeff: C1 factor (e.g. 22.37 for clamped-clamped, 9.87 for pinned-pinned)
public function span_natural_frequency_hz(
    atom span_len_m,
    atom D_outer_m,
    atom t_wall_m,
    atom m_eff_kg_m,
    atom boundary_coeff
)
    if span_len_m < 1.0 or m_eff_kg_m < 1.0 then
        return 999.0
    end if

    atom E = STEEL_YOUNGS_MODULUS_PA
    atom I = pipe_area_moment_inertia(D_outer_m, t_wall_m)
    atom L4 = span_len_m * span_len_m * span_len_m * span_len_m

    atom f0 = (boundary_coeff / TWO_PI) * sqrt((E * I) / (m_eff_kg_m * L4))
    return f0
end function

-- Calculate vortex shedding frequency (Hz) according to Strouhal formula:
-- f_v = St * v_current / D
public function span_vortex_shedding_frequency_hz(atom current_vel_m_s, atom D_outer_m)
    if D_outer_m < 0.05 then
        return 0.0
    end if
    return (STROUHAL_NUMBER_SUBSEA * current_vel_m_s) / D_outer_m
end function

-- Calculate reduced velocity V_R (dimensionless):
-- V_R = v_current / (f0 * D)
public function span_reduced_velocity(atom current_vel_m_s, atom f0_hz, atom D_outer_m)
    if f0_hz < 1.0e-4 or D_outer_m < 0.05 then
        return 0.0
    end if
    return current_vel_m_s / (f0_hz * D_outer_m)
end function

-- Assess free span VIV lock-in resonance risk according to DNV-RP-F105:
-- Returns sequence {is_lockin, viv_mode, reduced_velocity}
-- viv_mode: 0 = No VIV, 1 = In-line lock-in (1.0 <= V_R <= 4.5), 2 = Cross-flow lock-in (V_R > 4.5)
public function span_assess_viv_lockin(atom current_vel_m_s, atom f0_hz, atom D_outer_m)
    atom v_r = span_reduced_velocity(current_vel_m_s, f0_hz, D_outer_m)

    if v_r >= 4.5 then
        -- Critical cross-flow lock-in: large amplitude transverse vibrations
        return {1, 2, v_r}
    elsif v_r >= 1.0 then
        -- In-line lock-in: axial fatigue damage accumulation
        return {1, 1, v_r}
    else
        return {0, 0, v_r}
    end if
end function
