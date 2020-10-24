-- Moonpool Subsea Robotics Mission Scenarios
-- North Sea Subsea Pipeline Cathodic and Ultrasonic Autonomous Survey Simulation
-- OpenEuphoria v4.1+ Specification

namespace moonpool_scenario

include std/math.e
include std/console.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e
include src/core/quaternion.e
include src/acoustic/sound_speed.e
include src/acoustic/ray_trace.e
include src/acoustic/usbl.e
include src/navigation/dvl.e
include src/navigation/depth_sensor.e
include src/navigation/ekf.e
include src/bathymetry/dem_grid.e
include src/inspection/cathodic.e
include src/inspection/anode.e
include src/inspection/ultrasonic.e
include src/inspection/b31g.e
include src/control/hydrodynamics.e
include src/control/allocation.e
include src/control/guidance.e
include src/mission/executive.e

-- Run Scenario 1: Autonomous Subsea Pipeline Inspection and Integrity Assessment
-- Returns sequence {exec_state, sim_history, total_distance_m, defect_count, mission_success}
public function run_pipeline_survey_scenario(integer verbose)
    sequence exec
    atom dt, total_time, sim_t
    sequence veh_pos, veh_vel, veh_att
    sequence pipeline_start, pipeline_end
    sequence ekf, B_alloc, thruster_health
    atom total_dist
    integer defect_count
    sequence sim_history
    integer state
    sequence target_wpt, los_out
    atom cmd_yaw, fwd_speed
    sequence accel_raw, gyro_raw, ekf_nom
    atom cp_meas_mv, grad_meas
    integer holiday
    atom pit_depth, rem_thick, loss_pct, fpr
    sequence b31g_res

    exec = executive_create()
    exec = executive_log_event(exec, "MISSION INITIALIZED: Autonomous Pipeline Survey Lot 4B")

    -- 1. Pre-dive check
    exec = executive_transition(exec, STATE_MOONPOOL_DESCENT, "Pre-dive diagnostics verified 100%")

    -- Simulation parameters
    dt = 0.5
    total_time = 75.0
    sim_t = 0.0

    veh_pos = {0.0, 0.0, 5.0}
    veh_vel = {0.0, 0.0, 3.5}
    veh_att = quat_identity()

    pipeline_start = {0.0, 50.0, 100.0}
    pipeline_end   = {500.0, 50.0, 100.0}

    ekf = ekf_create(veh_pos, veh_vel, veh_att, 0.5, 0.05, 0.005)
    B_alloc = allocation_matrix_create(0.6, 0.4, 0.5, 0.3)
    thruster_health = repeat(1.0, 8)

    total_dist = 0.0
    defect_count = 0
    sim_history = {}

    -- Step simulation loop
    while sim_t < total_time do
        sim_t += dt
        exec[2] = sim_t
        exec[3] -= 0.03

        state = exec[1]

        if state = STATE_MOONPOOL_DESCENT and veh_pos[3] >= 15.0 then
            exec = executive_transition(exec, STATE_TRANSIT_TO_SEABED, "Cleared vessel hull boundary")
        elsif state = STATE_TRANSIT_TO_SEABED and veh_pos[3] >= 75.0 then
            exec = executive_transition(exec, STATE_ACOUSTIC_LOCK, "Entering seabed acoustic transponder envelope")
        elsif state = STATE_ACOUSTIC_LOCK and veh_pos[3] >= 94.0 then
            exec = executive_transition(exec, STATE_PIPELINE_TRACKING, "Acquired pipeline electromagnetic corridor")
        end if

        target_wpt = {veh_pos[1] + 30.0, 50.0, 95.0}
        if state = STATE_MOONPOOL_DESCENT or state = STATE_TRANSIT_TO_SEABED then
            target_wpt = {0.0, 50.0, 95.0}
        end if

        los_out = los_guidance_step(veh_pos, {0.0, 50.0, 95.0}, {500.0, 50.0, 95.0}, 10.0, 3.0)
        cmd_yaw = los_out[1]

        accel_raw = {0.0, 0.0, -GRAVITY_STANDARD}
        gyro_raw  = {0.0, 0.0, 0.0}

        ekf = ekf_predict(ekf, accel_raw, gyro_raw, dt)

        fwd_speed = 0.0
        if state >= STATE_PIPELINE_TRACKING and state < STATE_RETURN_TO_MOONPOOL then
            fwd_speed = 2.0
        end if
        ekf = ekf_update_dvl(ekf, {fwd_speed, 0.0, 0.0}, 0.02)
        ekf = ekf_update_depth(ekf, veh_pos[3], 0.01)

        if remainder(floor(sim_t * 2.0), 4) = 0 then
            ekf = ekf_update_position(ekf, veh_pos, 0.2)
        end if

        ekf_nom = ekf[1]
        veh_pos = ekf_nom[1]
        veh_vel = ekf_nom[2]

        if state = STATE_MOONPOOL_DESCENT or state = STATE_TRANSIT_TO_SEABED then
            veh_pos[3] += 5.5 * dt
            veh_pos[2] += 2.5 * dt
        elsif state = STATE_ACOUSTIC_LOCK then
            veh_pos[3] += 3.0 * dt
            veh_pos[2] = 50.0
        elsif state >= STATE_PIPELINE_TRACKING and state < STATE_RETURN_TO_MOONPOOL then
            veh_pos[1] += fwd_speed * dt
            veh_pos[2] = 50.0
            veh_pos[3] = 95.0
            total_dist += fwd_speed * dt
        end if

        -- Pipeline Cathodic & NDT Inspection Check at KP 20m
        if state = STATE_PIPELINE_TRACKING and veh_pos[1] >= 20.0 and defect_count = 0 then
            exec = executive_transition(exec, STATE_CP_INSPECTION, "CP potential drop detected")
            
            cp_meas_mv = -740.0
            grad_meas = cp_field_gradient(-950.0, cp_meas_mv, 0.5)
            holiday = cp_detect_coating_holiday(cp_meas_mv, grad_meas, 40.0)

            if holiday = 1 then
                exec = executive_transition(exec, STATE_NDT_SCANNING, "Coating holiday confirmed; deploying ultrasonic scanner")
                
                pit_depth = 4.2
                rem_thick = 19.1 - pit_depth
                loss_pct = (pit_depth / 19.1) * 100.0
                
                b31g_res = b31g_original_eval(609.6, 19.1, 448.0, 0.72, pit_depth, 120.0)
                fpr = b31g_res[3]

                exec = executive_record_defect(exec, {veh_pos[1], "PITTING_HOLIDAY", cp_meas_mv, pit_depth, loss_pct, fpr})
                defect_count += 1

                exec = executive_transition(exec, STATE_PIPELINE_TRACKING, "NDT scan recorded; resuming pipeline trajectory")
            end if
        end if

        if veh_pos[1] >= 35.0 and state != STATE_RETURN_TO_MOONPOOL and state != STATE_MISSION_COMPLETE then
            exec = executive_transition(exec, STATE_RETURN_TO_MOONPOOL, "Target survey length achieved")
        end if

        if state = STATE_RETURN_TO_MOONPOOL and veh_pos[3] <= 6.0 then
            exec = executive_transition(exec, STATE_MISSION_COMPLETE, "Safely docked in mothership moonpool latch")
            exit
        end if

        if state = STATE_RETURN_TO_MOONPOOL then
            veh_pos[3] -= 6.0 * dt
        end if

        if verbose = 1 and remainder(floor(sim_t), 4) = 0 and remainder(floor(sim_t * 2.0), 2) = 0 then
            printf(1, "  [t=%4.1fs] State=%-18s Pos={N:%.1f, E:%.1f, D:%.1f} Bat=%.1f%%\n", {
                sim_t, state_name(exec[1]), veh_pos[1], veh_pos[2], veh_pos[3], exec[3]
            })
        end if

        sim_history = append(sim_history, {sim_t, exec[1], veh_pos[1], veh_pos[2], veh_pos[3]})
    end while

    return {exec, sim_history, total_dist, defect_count, 1}
end function
