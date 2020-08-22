-- Moonpool Subsea Robotics Pipeline Inspection
-- Cathodic Protection (CP) Electric Field Potential Analysis and Coating Holiday Detection
-- OpenEuphoria v4.1+ Specification

namespace moonpool_cathodic

include std/math.e
include src/core/constants.e
include src/core/math_util.e

-- NACE SP0169 / ISO 15589-2 Standard CP Status Codes
public constant CP_STATUS_ADEQUATE         = 1
public constant CP_STATUS_UNDERPROTECTED    = 2
public constant CP_STATUS_OVERPROTECTED     = 3
public constant CP_STATUS_COATING_HOLIDAY   = 4

-- NACE SP0169 Thresholds for Carbon Steel with Ag/AgCl reference electrode in seawater (mV)
public constant CP_MIN_PROTECTION_MV = -800.0   -- Less negative than -800 mV indicates underprotection
public constant CP_OPTIMAL_MV        = -950.0   -- Ideal polarization potential
public constant CP_MAX_PROTECTION_MV = -1100.0  -- More negative than -1100 mV risks hydrogen embrittlement

-- Evaluate cathodic protection status from measured potential (millivolts vs Ag/AgCl)
-- Returns status code {status_code, description_string}
public function cp_evaluate_potential(atom potential_mv)
    if potential_mv > CP_MIN_PROTECTION_MV then
        -- e.g. -650 mV (underprotected, active corrosion risk)
        return {CP_STATUS_UNDERPROTECTED, "UNDERPROTECTED: Active corrosion risk (< -800 mV)"}
    elsif potential_mv < CP_MAX_PROTECTION_MV then
        -- e.g. -1150 mV (overprotected, hydrogen embrittlement / disbondment risk)
        return {CP_STATUS_OVERPROTECTED, "OVERPROTECTED: Hydrogen embrittlement risk (> -1100 mV)"}
    else
        return {CP_STATUS_ADEQUATE, "ADEQUATE: Polarization within NACE SP0169 bounds"}
    end if
end function

-- Calculate electric field gradient along pipeline trajectory
-- phi1_mv, phi2_mv: sequential potential measurements (mV)
-- dx_m: axial distance between sensor sampling points (meters)
-- Returns electric field gradient in microvolts per centimeter (uV/cm)
public function cp_field_gradient(atom phi1_mv, atom phi2_mv, atom dx_m)
    if dx_m < 0.01 then
        return 0.0
    end if
    -- Convert mV/m to uV/cm: 1 mV/m = 1000 uV / 100 cm = 10 uV/cm
    atom d_phi_mv = phi2_mv - phi1_mv
    atom grad_mv_per_m = d_phi_mv / dx_m
    return grad_mv_per_m * 10.0
end function

-- Detect coating holidays (defects) from localized potential dip and high electric field gradient
-- A holiday causes a sharp local anodic potential drop toward bare steel potential (-650 mV)
-- and high radial electric field entering the defect
public function cp_detect_coating_holiday(
    atom potential_mv,
    atom field_gradient_uv_cm,
    atom gradient_threshold_uv_cm
)
    if potential_mv > -850.0 and abs(field_gradient_uv_cm) >= gradient_threshold_uv_cm then
        return 1 -- Coating holiday detected
    else
        return 0
    end if
end function
