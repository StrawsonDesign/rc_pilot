/**
 * @file setpoint_manager.c
 *
 *
 **/
#include <math.h>
#include <stdio.h>
#include <string.h>  // for memset

#include <rc/start_stop.h>

#include <feedback.h>
#include <flight_mode.h>
#include <input_manager.h>
#include <rc_pilot_defs.h>
#include <setpoint_manager.h>
#include <settings.h>
#include <state_estimator.h>

#define XYZ_MAX_ERROR 0.5  ///< meters.

setpoint_t setpoint;  // extern variable in setpoint_manager.h

void __update_yaw(void)
{
    // if throttle stick is down all the way, probably landed, so
    // keep the yaw setpoint at current yaw so it takes off straight
    if (user_input.thr_stick < -0.95)
    {
        setpoint.yaw = state_estimate.yaw;
        setpoint.yaw_dot = 0.0;
        return;
    }
    // otherwise, scale yaw_rate by max yaw rate in rad/s
    // and move yaw setpoint
    setpoint.yaw_dot = user_input.yaw_stick * MAX_YAW_RATE;
    setpoint.yaw += setpoint.yaw_dot * DT;
    return;
}

void __update_Z(void)
{
    // make sure setpoint doesn't go too far below current altitude since we
    // can't sink into the ground
    if (setpoint.Z > (state_estimate.Z + XYZ_MAX_ERROR))
    {
        setpoint.Z = state_estimate.Z + XYZ_MAX_ERROR;
        setpoint.Z_dot = 0.0;
        return;
    }
    setpoint.Z_dot = -user_input.thr_stick * settings.max_Z_velocity;
    setpoint.Z += setpoint.Z_dot * DT;
    return;
}

void __update_XY_pos(void)
{
    // make sure setpoint doesn't go too far from state in case touching something
    if (setpoint.X > (state_estimate.X + XYZ_MAX_ERROR))
    {
        setpoint.X = state_estimate.X + XYZ_MAX_ERROR;
        setpoint.X_dot = 0.0;
    }
    else if (setpoint.X < (state_estimate.X - XYZ_MAX_ERROR))
    {
        setpoint.X = state_estimate.X - XYZ_MAX_ERROR;
        setpoint.X_dot = 0.0;
        return;
    }
    else
    {
        setpoint.X += setpoint.X_dot * DT;
    }

    if (setpoint.Y > (state_estimate.Y + XYZ_MAX_ERROR))
    {
        setpoint.Y = state_estimate.Y + XYZ_MAX_ERROR;
        setpoint.Y_dot = 0.0;
        return;
    }
    else if (setpoint.Y < (state_estimate.Y - XYZ_MAX_ERROR))
    {
        setpoint.Y = state_estimate.Y - XYZ_MAX_ERROR;
        setpoint.Y_dot = 0.0;
        return;
    }
    else
    {
        setpoint.Y += setpoint.Y_dot * DT;
    }

    return;
}

int setpoint_manager_init(void)
{
    if (setpoint.initialized)
    {
        fprintf(stderr, "ERROR in setpoint_manager_init, already initialized\n");
        return -1;
    }
    memset(&setpoint, 0, sizeof(setpoint_t));
    setpoint.initialized = 1;
    return 0;
}

int setpoint_manager_update(void)
{
    if (setpoint.initialized == 0)
    {
        fprintf(stderr, "ERROR in setpoint_manager_update, not initialized yet\n");
        return -1;
    }

    if (user_input.initialized == 0)
    {
        fprintf(stderr, "ERROR in setpoint_manager_update, input_manager not initialized yet\n");
        return -1;
    }

    // if PAUSED or UNINITIALIZED, do nothing
    if (rc_get_state() != RUNNING) return 0;

    // shutdown feedback on kill switch
    if (user_input.requested_arm_mode == DISARMED)
    {
        if (fstate.arm_state == ARMED) feedback_disarm();
        return 0;
    }

    // finally, switch between flight modes and adjust setpoint properly
    switch (user_input.flight_mode)
    {
        case TEST_BENCH_4DOF:
            // configure which controllers are enabled
            setpoint.en_6dof = 0;
            setpoint.en_rpy_ctrl = 0;
            setpoint.en_Z_ctrl = 0;
            setpoint.en_XY_vel_ctrl = 0;
            setpoint.en_XY_pos_ctrl = 0;

            setpoint.roll_throttle = user_input.roll_stick;
            setpoint.pitch_throttle = user_input.pitch_stick;
            setpoint.yaw_throttle = user_input.yaw_stick;
            setpoint.Z_throttle = -user_input.thr_stick;
            // TODO add these two throttle modes as options to settings, I use a radio
            // with self-centering throttle so having 0 in the middle is safest
            // setpoint.Z_throttle = -(user_input.thr_stick+1.0)/2.0;
            break;

        case TEST_BENCH_6DOF:
            setpoint.en_6dof = 1;
            setpoint.en_rpy_ctrl = 0;
            setpoint.en_Z_ctrl = 0;
            setpoint.en_XY_vel_ctrl = 0;
            setpoint.en_XY_pos_ctrl = 0;

            setpoint.X_throttle = -user_input.pitch_stick;
            setpoint.Y_throttle = user_input.roll_stick;
            setpoint.roll_throttle = 0.0;
            setpoint.pitch_throttle = 0.0;
            setpoint.yaw_throttle = user_input.yaw_stick;
            setpoint.Z_throttle = -user_input.thr_stick;
            break;

        case DIRECT_THROTTLE_4DOF:
            setpoint.en_6dof = 0;
            setpoint.en_rpy_ctrl = 1;
            setpoint.en_Z_ctrl = 0;
            setpoint.en_XY_vel_ctrl = 0;
            setpoint.en_XY_pos_ctrl = 0;

            setpoint.roll = user_input.roll_stick;
            setpoint.pitch = user_input.pitch_stick;
            setpoint.Z_throttle = -user_input.thr_stick;
            __update_yaw();
            break;

        case DIRECT_THROTTLE_6DOF:
            setpoint.en_6dof = 1;
            setpoint.en_rpy_ctrl = 1;
            setpoint.en_Z_ctrl = 0;
            setpoint.en_XY_vel_ctrl = 0;
            setpoint.en_XY_pos_ctrl = 0;

            setpoint.X_throttle = -user_input.pitch_stick;
            setpoint.Y_throttle = user_input.roll_stick;
            setpoint.Z_throttle = -user_input.thr_stick;
            __update_yaw();
            break;

        case ALT_HOLD_4DOF:
            setpoint.en_6dof = 0;
            setpoint.en_rpy_ctrl = 1;
            setpoint.en_Z_ctrl = 1;
            setpoint.en_XY_vel_ctrl = 0;
            setpoint.en_XY_pos_ctrl = 0;

            setpoint.roll = user_input.roll_stick;
            setpoint.pitch = user_input.pitch_stick;
            __update_Z();
            __update_yaw();
            break;

        case ALT_HOLD_6DOF:
            setpoint.en_6dof = 1;
            setpoint.en_rpy_ctrl = 1;
            setpoint.en_Z_ctrl = 1;
            setpoint.en_XY_vel_ctrl = 0;
            setpoint.en_XY_pos_ctrl = 0;

            setpoint.roll = 0.0;
            setpoint.pitch = 0.0;
            setpoint.X_throttle = -user_input.pitch_stick;
            setpoint.Y_throttle = user_input.roll_stick;
            __update_Z();
            __update_yaw();
            break;

        case VELOCITY_CONTROL_4DOF:
            setpoint.en_6dof = 0;
            setpoint.en_rpy_ctrl = 1;
            setpoint.en_Z_ctrl = 1;
            setpoint.en_XY_vel_ctrl = 1;
            setpoint.en_XY_pos_ctrl = 0;

            setpoint.X_dot = -user_input.pitch_stick * settings.max_XY_velocity;
            setpoint.Y_dot = user_input.roll_stick * settings.max_XY_velocity;
            __update_Z();
            __update_yaw();
            break;

        case VELOCITY_CONTROL_6DOF:
            setpoint.en_6dof = 1;
            setpoint.en_rpy_ctrl = 1;
            setpoint.en_Z_ctrl = 1;
            setpoint.en_XY_vel_ctrl = 1;
            setpoint.en_XY_pos_ctrl = 0;

            setpoint.X_dot = -user_input.pitch_stick * settings.max_XY_velocity;
            setpoint.Y_dot = user_input.roll_stick * settings.max_XY_velocity;
            __update_Z();
            __update_yaw();
            break;

        case POSITION_CONTROL_4DOF:
            setpoint.en_6dof = 0;
            setpoint.en_rpy_ctrl = 1;
            setpoint.en_Z_ctrl = 1;
            setpoint.en_XY_vel_ctrl = 0;
            setpoint.en_XY_pos_ctrl = 1;

            setpoint.X_dot = -user_input.pitch_stick * settings.max_XY_velocity;
            setpoint.Y_dot = user_input.roll_stick * settings.max_XY_velocity;
            __update_XY_pos();
            __update_Z();
            __update_yaw();
            break;

        case POSITION_CONTROL_6DOF:
            setpoint.en_6dof = 1;
            setpoint.en_rpy_ctrl = 1;
            setpoint.en_Z_ctrl = 1;
            setpoint.en_XY_vel_ctrl = 0;
            setpoint.en_XY_pos_ctrl = 1;

            setpoint.X_dot = -user_input.pitch_stick * settings.max_XY_velocity;
            setpoint.Y_dot = user_input.roll_stick * settings.max_XY_velocity;
            __update_XY_pos();
            __update_Z();
            __update_yaw();
            break;

        default:  // should never get here
            fprintf(stderr, "ERROR in setpoint_manager thread, unknown flight mode\n");
            break;

    }  // end switch(user_input.flight_mode)

    // arm feedback when requested
    if (user_input.requested_arm_mode == ARMED)
    {
        if (fstate.arm_state == DISARMED) feedback_arm();
    }

    return 0;
}

int setpoint_manager_cleanup(void)
{
    setpoint.initialized = 0;
    return 0;
}
