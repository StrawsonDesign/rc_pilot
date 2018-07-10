/**
* @file setpoint_manager.c
*
*
**/
#include <stdio.h>
#include <math.h>
#include <string.h> // for memset

#include <rc/start_stop.h>

#include <setpoint_manager.h>
#include <settings.h>
#include <input_manager.h>
#include <feedback.h>
#include <rc_pilot_defs.h>
#include <flight_mode.h>


setpoint_t setpoint; // extern variable in setpoint_manager.h

void __direct_throttle()
{
	double tmp;
	// translate throttle stick (-1,1) to throttle (0,1)
	// then scale and shift to be between thrust min/max
	// Z-throttle should be negative since Z points down
	tmp = (user_input.thr_stick + 1.0)/2.0;
	//tmp = tmp * (MAX_Z_COMPONENT - MIN_Z_COMPONENT);
	setpoint.Z_throttle = -tmp;
	return;
}

void __direct_yaw()
{
	// if throttle stick is down all the way, probably landed, so
	// keep the yaw setpoint at current yaw so it takes off straight
	if(user_input.thr_stick < -0.95){
		setpoint.yaw = fstate.yaw;
		setpoint.yaw_rate = 0.0;
	}
	// otherwise, scale yaw_rate by max yaw rate in rad/s
	// and move yaw setpoint
	else{
		setpoint.yaw_rate = user_input.yaw_stick * MAX_YAW_RATE;
		setpoint.yaw_rate += setpoint.yaw_rate/settings.feedback_hz;
	}
	return;
}

int setpoint_manager_init()
{
	if(setpoint.initialized){
		fprintf(stderr, "ERROR in setpoint_manager_init, already initialized\n");
		return -1;
	}
	memset(&setpoint,0,sizeof(setpoint_t));
	setpoint.initialized = 1;
	return 0;
}



int setpoint_manager_update()
{
	if(setpoint.initialized==0){
		fprintf(stderr, "ERROR in setpoint_manager_update, not initialized yet\n");
		return -1;
	}

	if(user_input.initialized==0){
		fprintf(stderr, "ERROR in setpoint_manager_update, input_manager not initialized yet\n");
		return -1;
	}

	// if PAUSED or UNINITIALIZED, do nothing
	if(rc_get_state()!=RUNNING) return 0;

	// shutdown feedback on kill switch
	if(user_input.requested_arm_mode == DISARMED){
		if(fstate.arm_state==ARMED) feedback_disarm();
		return 0;
	}

	// finally, switch between flight modes and adjust setpoint properly
	switch(user_input.flight_mode){


	//TODO: pitch doesn't work in this mode... why? 
	case TEST_BENCH_4DOF:
		setpoint.en_alt_ctrl = 0;
		setpoint.en_rpy_ctrl = 0;
		setpoint.en_6dof = 0;
		setpoint.roll_throttle = user_input.roll_stick;
		setpoint.pitch_throttle = user_input.pitch_stick;
		setpoint.yaw_throttle = user_input.yaw_stick;
		setpoint.Z_throttle = -(user_input.thr_stick+1.0)/2.0;
		break;

	case TEST_BENCH_6DOF:
		setpoint.en_alt_ctrl = 0;
		setpoint.en_rpy_ctrl = 0;
		setpoint.en_6dof = 1;
		setpoint.X_throttle = -user_input.pitch_stick;
		setpoint.Y_throttle = user_input.roll_stick;
		setpoint.roll_throttle = 0.0;
		setpoint.pitch_throttle = 0.0;
		setpoint.yaw_throttle = user_input.yaw_stick;
		setpoint.Z_throttle = -user_input.thr_stick;
		break;

	case DIRECT_THROTTLE_4DOF:
		setpoint.en_alt_ctrl = 0;
		setpoint.en_rpy_ctrl = 1;
		setpoint.en_6dof = 0;
		setpoint.roll = user_input.roll_stick;
		setpoint.pitch = user_input.pitch_stick;
		__direct_throttle();
		__direct_yaw();
		setpoint.altitude = setpoint.Z_throttle;
		break;

	case DIRECT_THROTTLE_6DOF:
		setpoint.en_alt_ctrl = 0;
		setpoint.en_rpy_ctrl = 1;
		setpoint.en_6dof = 0;
		setpoint.roll = 0.0;
		setpoint.pitch = 0.0;
		setpoint.X_throttle = -user_input.pitch_stick;
		setpoint.Y_throttle = user_input.roll_stick;
		__direct_throttle();
		__direct_yaw();
		break;

	case ALT_HOLD_4DOF:
		setpoint.en_alt_ctrl = 0;
		setpoint.en_rpy_ctrl = 1;
		setpoint.en_6dof = 0;
		setpoint.roll = user_input.roll_stick;
		setpoint.pitch = user_input.pitch_stick;
		__direct_throttle();
		__direct_yaw();
		break;

	case ALT_HOLD_6DOF:
		setpoint.en_alt_ctrl = 0;
		setpoint.en_rpy_ctrl = 1;
		setpoint.en_6dof = 0;
		setpoint.roll = 0.0;
		setpoint.pitch = 0.0;
		setpoint.X_throttle = -user_input.pitch_stick;
		setpoint.Y_throttle = user_input.roll_stick;
		__direct_throttle();
		__direct_yaw();
		break;

	default: // should never get here
		fprintf(stderr,"ERROR in setpoint_manager thread, unknown flight mode\n");
		break;

	} // end switch(user_input.flight_mode)

	// arm feedback when requested
	if(user_input.requested_arm_mode == ARMED){
		if(fstate.arm_state==DISARMED) feedback_arm();
	}

	return 0;
}


int setpoint_manager_cleanup()
{
	setpoint.initialized=0;
	return 0;
}
