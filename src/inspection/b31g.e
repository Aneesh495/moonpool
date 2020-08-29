-- Moonpool Subsea Robotics Pipeline Inspection
-- ASME B31G and Modified B31G (0.85dL RSTRENG) Fitness-for-Service Burst Pressure Analysis
-- OpenEuphoria v4.1+ Specification

namespace moonpool_b31g

include std/math.e
include src/core/constants.e
include src/core/math_util.e

-- ASME B31G Integrity Action Codes
public constant B31G_STATUS_ACCEPTABLE  = 1 -- Safe to operate at MAOP
public constant B31G_STATUS_MONITOR     = 2 -- Safe, scheduled future re-inspection
public constant B31G_STATUS_DERATE      = 3 -- Requires operating pressure de-rating
public constant B31G_STATUS_REPAIR      = 4 -- Unsafe: immediate repair or sleeve required

-- Evaluate pipeline defect severity using original ASME B31G standard:
-- D_mm: pipe outer diameter (mm)
-- t_mm: nominal wall thickness (mm)
-- smys_mpa: Specified Minimum Yield Strength (e.g. 448 MPa for API 5L X65)
-- f_design: pipeline design factor (e.g. 0.72)
-- d_mm: measured corrosion pit depth (mm)
-- L_mm: axial defect length (mm)
-- Returns sequence {p_safe_bar, p_maop_bar, failure_pressure_ratio, status_code}
public function b31g_original_eval(
    atom D_mm,
    atom t_mm,
    atom smys_mpa,
    atom f_design,
    atom d_mm,
    atom L_mm
)
    atom p_maop_mpa, p_safe_mpa, fpr, m_factor, z_param
    atom dt_ratio
    integer status

    -- Base MAOP (Barlow's formula with design factor): P = (2 * S * t / D) * F (MPa)
    p_maop_mpa = (2.0 * smys_mpa * (t_mm / D_mm)) * f_design
    atom p_maop_bar = p_maop_mpa * 10.0 -- 1 MPa = 10 bar

    dt_ratio = d_mm / t_mm

    -- Depth check criteria
    if dt_ratio <= 0.10 then
        -- Negligible corrosion loss
        return {p_maop_bar, p_maop_bar, 1.0, B31G_STATUS_ACCEPTABLE}
    elsif dt_ratio >= 0.80 then
        -- Critical limit: defects exceeding 80% wall thickness require immediate repair
        return {0.0, p_maop_bar, 0.0, B31G_STATUS_REPAIR}
    end if

    -- Geometric parameter: A = 0.893 * L / sqrt(D * t)
    z_param = (L_mm * L_mm) / (D_mm * t_mm)
    atom A_param = 0.893 * L_mm / sqrt(D_mm * t_mm)

    if A_param <= 4.0 then
        -- Short defect formulation
        m_factor = sqrt(1.0 + 0.893 * z_param)
        atom num = 1.0 - (2.0 / 3.0) * dt_ratio
        atom den = 1.0 - (2.0 / 3.0) * (dt_ratio / m_factor)
        p_safe_mpa = 1.1 * p_maop_mpa * (num / den)
    else
        -- Long defect formulation
        p_safe_mpa = 1.1 * p_maop_mpa * (1.0 - dt_ratio)
    end if

    -- Safe pressure cannot exceed original design MAOP
    if p_safe_mpa > p_maop_mpa then
        p_safe_mpa = p_maop_mpa
    end if

    atom p_safe_bar = p_safe_mpa * 10.0
    fpr = p_safe_mpa / p_maop_mpa

    if fpr >= 1.0 then
        status = B31G_STATUS_ACCEPTABLE
    elsif fpr >= 0.90 then
        status = B31G_STATUS_MONITOR
    elsif fpr >= 0.70 then
        status = B31G_STATUS_DERATE
    else
        status = B31G_STATUS_REPAIR
    end if

    return {p_safe_bar, p_maop_bar, fpr, status}
end function

-- Evaluate pipeline defect severity using Modified ASME B31G (0.85dL / RSTRENG):
-- Uses flow stress sigma_flow = SMYS + 68.95 MPa and 0.85 parabolic area factor
public function b31g_modified_eval(
    atom D_mm,
    atom t_mm,
    atom smys_mpa,
    atom f_design,
    atom d_mm,
    atom L_mm
)
    atom sigma_flow_mpa = smys_mpa + 68.95
    atom p_maop_mpa = (2.0 * smys_mpa * (t_mm / D_mm)) * f_design
    atom p_maop_bar = p_maop_mpa * 10.0

    atom dt_ratio = d_mm / t_mm
    if dt_ratio <= 0.10 then
        return {p_maop_bar, p_maop_bar, 1.0, B31G_STATUS_ACCEPTABLE}
    elsif dt_ratio >= 0.80 then
        return {0.0, p_maop_bar, 0.0, B31G_STATUS_REPAIR}
    end if

    atom z_param = (L_mm * L_mm) / (D_mm * t_mm)
    atom m_factor

    if z_param <= 50.0 then
        m_factor = sqrt(1.0 + 0.6275 * z_param - 0.003375 * (z_param * z_param))
    else
        m_factor = 0.032 * z_param + 3.3
    end if

    atom num = 1.0 - 0.85 * dt_ratio
    atom den = 1.0 - 0.85 * (dt_ratio / m_factor)

    -- Burst pressure calculation
    atom p_burst_mpa = (2.0 * sigma_flow_mpa * (t_mm / D_mm)) * (num / den)
    atom p_safe_mpa = p_burst_mpa * f_design

    if p_safe_mpa > p_maop_mpa then
        p_safe_mpa = p_maop_mpa
    end if

    atom p_safe_bar = p_safe_mpa * 10.0
    atom fpr = p_safe_mpa / p_maop_mpa
    integer status

    if fpr >= 1.0 then
        status = B31G_STATUS_ACCEPTABLE
    elsif fpr >= 0.90 then
        status = B31G_STATUS_MONITOR
    elsif fpr >= 0.70 then
        status = B31G_STATUS_DERATE
    else
        status = B31G_STATUS_REPAIR
    end if

    return {p_safe_bar, p_maop_bar, fpr, status}
end function
