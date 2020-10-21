-- Moonpool Subsea Robotics Mission Executive
-- Autonomous State Machine, Subsea Intervention Lifecycle, and Telemetry Event Logging
-- OpenEuphoria v4.1+ Specification

namespace moonpool_executive

include std/math.e
include std/console.e
include src/core/constants.e
include src/core/math_util.e

-- Subsea Mission State Machine States
public constant STATE_PRE_DIVE_CHECK       = 1
public constant STATE_MOONPOOL_DESCENT     = 2
public constant STATE_TRANSIT_TO_SEABED    = 3
public constant STATE_ACOUSTIC_LOCK        = 4
public constant STATE_PIPELINE_TRACKING    = 5
public constant STATE_CP_INSPECTION        = 6
public constant STATE_NDT_SCANNING         = 7
public constant STATE_RETURN_TO_MOONPOOL   = 8
public constant STATE_MISSION_COMPLETE     = 9
public constant STATE_EMERGENCY_ABORT      = 10

-- State Name Strings for Telemetry Display
public function state_name(integer state_id)
    if state_id = STATE_PRE_DIVE_CHECK then
        return "PRE_DIVE_CHECK"
    elsif state_id = STATE_MOONPOOL_DESCENT then
        return "MOONPOOL_DESCENT"
    elsif state_id = STATE_TRANSIT_TO_SEABED then
        return "TRANSIT_TO_SEABED"
    elsif state_id = STATE_ACOUSTIC_LOCK then
        return "ACOUSTIC_LOCK"
    elsif state_id = STATE_PIPELINE_TRACKING then
        return "PIPELINE_TRACKING"
    elsif state_id = STATE_CP_INSPECTION then
        return "CP_INSPECTION"
    elsif state_id = STATE_NDT_SCANNING then
        return "NDT_SCANNING"
    elsif state_id = STATE_RETURN_TO_MOONPOOL then
        return "RETURN_TO_MOONPOOL"
    elsif state_id = STATE_MISSION_COMPLETE then
        return "MISSION_COMPLETE"
    elsif state_id = STATE_EMERGENCY_ABORT then
        return "EMERGENCY_ABORT"
    else
        return "UNKNOWN_STATE"
    end if
end function

-- Create a Mission Executive structure:
-- {current_state, mission_time_sec, battery_pct, defects_logged, event_log}
public function executive_create()
    return {STATE_PRE_DIVE_CHECK, 0.0, 100.0, {}, {}}
end function

-- Log a mission event to executive audit trail
public function executive_log_event(sequence exec, sequence msg)
    sequence events
    atom t
    sequence log_item

    events = exec[5]
    t = exec[2]
    log_item = {t, exec[1], msg}
    events = append(events, log_item)
    exec[5] = events
    return exec
end function

-- Transition executive state machine
public function executive_transition(sequence exec, integer new_state, sequence reason)
    integer old_state
    sequence msg

    old_state = exec[1]
    exec[1] = new_state
    msg = sprintf("TRANSITION: %s -> %s (Reason: %s)", {
        state_name(old_state),
        state_name(new_state),
        reason
    })
    return executive_log_event(exec, msg)
end function

-- Record an inspected pipeline defect event:
-- defect: {kp_m, defect_type, potential_mv, pit_depth_mm, wall_loss_pct, b31g_fpr}
public function executive_record_defect(sequence exec, sequence defect)
    sequence defects
    sequence msg

    defects = exec[4]
    defects = append(defects, defect)
    exec[4] = defects
    msg = sprintf("DEFECT DETECTED at KP %.1fm: Pit=%.1fmm (%.1f%% loss), B31G FPR=%.2f", {
        defect[1], defect[4], defect[5], defect[6]
    })
    return executive_log_event(exec, msg)
end function
