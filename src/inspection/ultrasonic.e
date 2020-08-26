-- Moonpool Subsea Robotics Pipeline Inspection
-- Multi-Channel Ultrasonic Pulse-Echo Non-Destructive Testing (NDT) Wall Thickness and Pitting
-- OpenEuphoria v4.1+ Specification

namespace moonpool_ultrasonic

include std/math.e
include src/core/constants.e
include src/core/math_util.e

-- Ultrasonic Defect Classification Severity Codes
public constant NDT_DEFECT_NONE             = 0
public constant NDT_DEFECT_LIGHT_CORROSION  = 1 -- < 10% wall loss
public constant NDT_DEFECT_MODERATE_PITTING = 2 -- 10% to 30% wall loss
public constant NDT_DEFECT_SEVERE_PITTING   = 3 -- 30% to 50% wall loss
public constant NDT_DEFECT_CRITICAL_LOSS    = 4 -- > 50% wall loss (immediate repair)

-- Calculate steel wall thickness from ultrasonic pulse-echo time-of-flight:
-- twtt_sec: acoustic two-way travel time between outer pipe surface and inner back-wall (seconds)
-- c_steel_m_s: longitudinal wave speed in steel (m/s, standard 5900 m/s)
-- Returns wall thickness in millimeters (mm)
public function ut_wall_thickness_mm(atom twtt_sec, atom c_steel_m_s)
    if twtt_sec <= 0.0 or c_steel_m_s <= 0.0 then
        return 0.0
    end if
    -- t = 0.5 * twtt * c (meters) -> multiply by 1000 for mm
    return 0.5 * twtt_sec * c_steel_m_s * 1000.0
end function

-- Calculate corrosion pit depth and remaining wall percentage:
-- t_nominal_mm: original nominal pipeline wall thickness (mm)
-- t_measured_mm: current ultrasonic wall thickness measurement (mm)
-- Returns sequence {pit_depth_mm, wall_loss_percentage, severity_code}
public function ut_analyze_corrosion(atom t_nominal_mm, atom t_measured_mm)
    atom pit_depth_mm = t_nominal_mm - t_measured_mm
    if pit_depth_mm < 0.0 then
        pit_depth_mm = 0.0 -- Manufacturing tolerance positive variance
    end if

    atom loss_pct = (pit_depth_mm / t_nominal_mm) * 100.0
    integer severity = NDT_DEFECT_NONE

    if loss_pct < 10.0 then
        severity = NDT_DEFECT_LIGHT_CORROSION
    elsif loss_pct < 30.0 then
        severity = NDT_DEFECT_MODERATE_PITTING
    elsif loss_pct < 50.0 then
        severity = NDT_DEFECT_SEVERE_PITTING
    else
        severity = NDT_DEFECT_CRITICAL_LOSS
    end if

    return {pit_depth_mm, loss_pct, severity}
end function

-- Analyze multi-channel circumferential ultrasonic transducer ring:
-- twtt_array: sequence of TWTT readings from 8 or 16 sensors around pipe clock positions
-- clock_labels: e.g. {"12:00", "01:30", "03:00", ..., "10:30"}
-- Returns sequence {min_thickness_mm, max_pit_depth_mm, critical_clock_pos, defect_list}
public function ut_scan_circumference(sequence twtt_array, atom t_nominal_mm, atom c_steel_m_s)
    integer n_sensors = length(twtt_array)
    atom min_thick = 999.0
    atom max_pit = 0.0
    integer crit_idx = 1
    sequence defects = {}

    for i = 1 to n_sensors do
        atom thick = ut_wall_thickness_mm(twtt_array[i], c_steel_m_s)
        sequence corr = ut_analyze_corrosion(t_nominal_mm, thick)
        atom pit = corr[1]
        atom loss_pct = corr[2]
        integer sev = corr[3]

        if thick < min_thick then
            min_thick = thick
            max_pit = pit
            crit_idx = i
        end if

        if sev >= NDT_DEFECT_MODERATE_PITTING then
            -- Record defect event: {sensor_index, thickness_mm, pit_depth_mm, loss_pct, severity}
            defects = append(defects, {i, thick, pit, loss_pct, sev})
        end if
    end for

    return {min_thick, max_pit, crit_idx, defects}
end function
