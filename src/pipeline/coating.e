-- Moonpool Subsea Robotics Pipeline Engineering
-- Pipeline Anticorrosion Coating Breakdown Kinetics and Cathodic Disbondment
-- OpenEuphoria v4.1+ Specification

namespace moonpool_coating

include std/math.e
include src/core/constants.e
include src/core/math_util.e

public constant COATING_TYPE_FBE    = 1 -- Fusion-Bonded Epoxy
public constant COATING_TYPE_3LPE   = 2 -- 3-Layer Polyethylene
public constant COATING_TYPE_3LPP   = 3 -- 3-Layer Polypropylene
public constant COATING_TYPE_ASPHALT= 4 -- Asphalt Enamel

-- Calculate DNV-RP-F103 coating breakdown factor f_b as a function of service years:
-- f_b = a + b * t
-- Returns dimensionless breakdown fraction [0.0, 1.0]
public function coating_breakdown_factor(integer coating_type, atom service_years)
    atom a_init, b_rate
    if coating_type = COATING_TYPE_3LPP then
        a_init = 0.005
        b_rate = 0.0005 -- Extremely high resistance
    elsif coating_type = COATING_TYPE_3LPE then
        a_init = 0.010
        b_rate = 0.0010
    elsif coating_type = COATING_TYPE_FBE then
        a_init = 0.020
        b_rate = 0.0025
    else
        a_init = 0.050
        b_rate = 0.0080 -- Asphalt/coal tar enamel
    end if

    atom f_b = a_init + b_rate * service_years
    return clamp(f_b, 0.0, 1.0)
end function

-- Calculate cathodic protection current density demand (mA/m^2):
-- i_mean = f_b * i_bare + (1 - f_b) * i_coated
-- i_bare_ma_m2: bare steel polarising current density (typically 80 to 120 mA/m^2 in North Sea)
-- i_coated_ma_m2: defect-free coated current density (typically 0.01 to 0.05 mA/m^2)
public function coating_current_demand(
    integer coating_type,
    atom service_years,
    atom i_bare_ma_m2,
    atom i_coated_ma_m2
)
    atom f_b = coating_breakdown_factor(coating_type, service_years)
    return f_b * i_bare_ma_m2 + (1.0 - f_b) * i_coated_ma_m2
end function

-- Calculate cathodic disbondment defect radius growth under overprotection:
-- r0_mm: initial holiday defect radius (mm)
-- potential_mv: polarization potential (e.g. -1150 mV accelerates disbondment)
-- temp_c: operating pipe temperature (deg C)
-- elapsed_days: operational exposure duration (days)
-- Returns updated disbondment radius in millimeters (mm)
public function coating_disbondment_radius_mm(
    atom r0_mm,
    atom potential_mv,
    atom temp_c,
    atom elapsed_days
)
    -- High overprotection (potential < -1100 mV) generates excessive OH- ions, promoting alkaline disbondment
    atom overprotection_delta = max({0.0, (-1100.0 - potential_mv)}) / 100.0
    atom temp_factor = 1.0 + 0.03 * (temp_c - 10.0)
    atom k_rate = (0.015 + 0.025 * overprotection_delta) * temp_factor

    atom delta_r = k_rate * sqrt(elapsed_days)
    return r0_mm + delta_r
end function
