-- Moonpool Subsea Robotics Pipeline Inspection
-- Sacrificial Anode (Al-Zn-In / Zinc) Electrochemical Consumption and Remaining Life Modeling
-- OpenEuphoria v4.1+ Specification

namespace moonpool_anode

include std/math.e
include src/core/constants.e
include src/core/math_util.e

-- Standard Electrochemical Properties of Offshore Sacrificial Anode Alloys
public constant ANODE_CAPACITY_ALUMINUM = 2000.0 -- Practical capacity of Al-Zn-In alloy (A*hr/kg)
public constant ANODE_CAPACITY_ZINC     = 780.0  -- Practical capacity of Zinc alloy (A*hr/kg)
public constant ANODE_POTENTIAL_AL_MV   = -1050.0 -- Closed circuit potential of Al alloy (mV vs Ag/AgCl)
public constant ANODE_POTENTIAL_ZN_MV   = -1030.0 -- Closed circuit potential of Zinc alloy (mV vs Ag/AgCl)
public constant ANODE_UTILIZATION_FACTOR = 0.80   -- DNV-RP-F103 standard bracelet utilization factor

-- Calculate anode resistance to remote earth using the McCoy formula:
-- For flush-mounted half-shell pipeline bracelet anodes
-- resistivity_ohm_m: seawater resistivity (typically 0.25 to 0.35 ohm*m in North Sea)
-- surface_area_m2: exposed anode active surface area (m^2)
public function anode_resistance_mccoy(atom resistivity_ohm_m, atom surface_area_m2)
    if surface_area_m2 <= 0.0 then
        return 999.0
    end if
    return (0.315 * resistivity_ohm_m) / sqrt(surface_area_m2)
end function

-- Calculate anode resistance to remote earth using Dwight's formula:
-- For cylindrical / stand-off offshore structural anodes
-- length_m: length of anode (meters)
-- radius_m: radius of anode cylinder (meters)
public function anode_resistance_dwight(atom resistivity_ohm_m, atom length_m, atom radius_m)
    if length_m <= 0.0 or radius_m <= 0.0 then
        return 999.0
    end if
    atom log_term = log(4.0 * length_m / radius_m) - 1.0
    return (resistivity_ohm_m / (TWO_PI * length_m)) * log_term
end function

-- Calculate current output delivered by sacrificial anode:
-- anode_pot_mv: closed-circuit anode potential (e.g. -1050 mV)
-- pipe_pot_mv: polarized pipeline potential (e.g. -900 mV)
-- resistance_ohm: anode resistance to remote earth (ohms)
-- Returns delivered current in Amperes (A)
public function anode_current_output(atom anode_pot_mv, atom pipe_pot_mv, atom resistance_ohm)
    if resistance_ohm < 1.0e-5 then
        resistance_ohm = 1.0e-5
    end if
    -- Driving potential: Delta E = E_pipe - E_anode (in Volts)
    atom driving_voltage = (pipe_pot_mv - anode_pot_mv) / 1000.0
    if driving_voltage < 0.0 then
        driving_voltage = 0.0
    end if
    return driving_voltage / resistance_ohm
end function

-- Calculate annual anode mass loss rate according to Faraday's law:
-- current_amps: delivered galvanic current (A)
-- capacity_a_hr_kg: alloy electrochemical capacity (A*hr/kg)
-- Returns mass loss rate in kg/year (8760 hours/year)
public function anode_consumption_rate_kg_yr(atom current_amps, atom capacity_a_hr_kg)
    if capacity_a_hr_kg <= 0.0 then
        return 0.0
    end if
    return (current_amps * 8760.0) / capacity_a_hr_kg
end function

-- Estimate remaining sacrificial anode operational lifespan in years:
-- current_mass_kg: measured or estimated remaining anode mass (kg)
-- initial_mass_kg: original nameplate anode mass (kg)
-- current_amps: present current output (A)
-- capacity_a_hr_kg: alloy electrochemical capacity (A*hr/kg)
-- utilization_factor: maximum usable mass fraction before detachment (e.g. 0.80)
public function anode_remaining_life_years(
    atom current_mass_kg,
    atom initial_mass_kg,
    atom current_amps,
    atom capacity_a_hr_kg,
    atom utilization_factor
)
    atom usable_initial_mass = initial_mass_kg * utilization_factor
    atom unconsumed_usable_mass = current_mass_kg - (initial_mass_kg * (1.0 - utilization_factor))
    if unconsumed_usable_mass <= 0.0 then
        return 0.0 -- Anode exhausted or below minimum structural attachment mass
    end if

    atom annual_loss = anode_consumption_rate_kg_yr(current_amps, capacity_a_hr_kg)
    if annual_loss < 1.0e-4 then
        return 100.0 -- Effectively unconsumed
    end if

    return unconsumed_usable_mass / annual_loss
end function
