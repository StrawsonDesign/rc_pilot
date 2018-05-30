/**
* @file setpoint_manager.c
*
*
**/
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>

#include <rc/state.h>

#include <setpoint_manager.h>
#include <fly_defs.h>


static setpoint_t sp;
static int initialized = 0;


int setpoint_manager_init()
{
	if(initialized){
		fprintf(stderr, "ERROR in setpoint_manager_init, already initialized\n");
		return -1;
	}
	memset(sp,0,sizeof(setpoint_t));
	initialized = 1;
	return 0;
}


int setpoint_manager_get_sp(setpoint_t* setpoint)
{
	if(initialized==0){
		fprintf(stderr, "ERROR in setpoint_manager_get_sp, not initialized\n");
		return -1;
	}
	if(setpoint==NULL){
		fprintf(stderr, "ERROR in setpoint_manager_get_sp, received NULL pointer\n");
		return -1;
	}
	setpoint* = sp;
	return 0;
}

int setpoint_manager_update()
{
	if(initialized==0){
		fprintf(stderr, "ERROR in setpoint_manager_update, not initialized\n");
		return -1;
	}

	// if PAUSED or UNINITIALIZED, do nothing
	if(rc_get_state()!=RUNNING) return 0;

	// if the core got disarmed, wait for arming sequence
	if(get_controller_arm_state() == DISARMED){
		wait_for_arming_sequence();
		// user may have pressed the pause button or shut down while waiting
		// check before continuing
		if(rc_get_state()!=RUNNING) continue;
		arm_controller();
		continue;
	}

	// shutdown core on kill switch
	if(ui->kill_switch == DISARMED){
		disarm_controller();
		continue;
	}

	// if user input is not active (radio disconnected) emergency land
	// TODO: hold still until battery is low enough to require landing
	//if(!ui->user_input_active) ui->flight_mode = EMERGENCY_LAND;

	// finally, switch between flight modes and adjust setpoint properly
	switch(ui->flight_mode){
	// Raw attitude mode lets user control the inner attitude loop directly
	case DIRECT_THROTTLE:
		sp->en_alt_ctrl = 0; // disable altitude controller
		sp->en_rpy_ctrl = 1;
		if(set->dof==6){
			sp->en_6dof = 1;
			sp->roll  = 0.0;
			sp->pitch = 0.0;
			sp->Y_throttle = ui->roll_stick;
			sp->X_throttle = ui->pitch_stick;
		}
		else{
			sp->en_6dof = 0;
			// scale roll and pitch angle by max setpoint in rad
			sp->roll  = ui->roll_stick  * MAX_ROLL_SETPOINT;
			sp->pitch = ui->pitch_stick * MAX_PITCH_SETPOINT;
			sp->Y_throttle = 0.0;
			sp->X_throttle = 0.0;
		}

		// translate throttle stick (-1,1) to throttle (0,1)
		// then scale and shift to be between thrust min/max
		// Z-throttle should be negative since Z points down
		tmp = (ui->thr_stick + 1.0)/2.0;
		tmp = tmp * (MAX_Z_COMPONENT - MIN_Z_COMPONENT);
		sp->Z_throttle = -(tmp + MIN_Z_COMPONENT);

		// if throttle stick is down all the way, probably landed, so
		// keep the yaw setpoint at current yaw so it takes off straight
		if(ui->thr_stick < -0.95){
			sp->yaw = cs->yaw;
			sp->yaw_rate = 0.0;
		}
		// otherwise, scale yaw_rate by max yaw rate in rad/s
		// also apply deadzone to prevent drift
		else{
			tmp = apply_deadzone(ui->yaw_stick, YAW_DEADZONE);
			sp->yaw_rate = tmp * MAX_YAW_RATE;
		}

		// done!
		break;

	// forces old 4DOF flight on 6dof frames
	case FALLBACK_4DOF:
		sp->en_alt_ctrl = 0; // disable altitude controller
		sp->en_6dof = 0;
		sp->en_rpy_ctrl = 1;
		// scale roll and pitch angle by max setpoint in rad
		sp->roll  = ui->roll_stick  * MAX_ROLL_SETPOINT;
		sp->pitch = ui->pitch_stick * MAX_PITCH_SETPOINT;
		sp->Y_throttle = 0.0;
		sp->X_throttle = 0.0;

		// translate throttle stick (-1,1) to throttle (0,1)
		// then scale and shift to be between thrust min/max
		// Z-throttle should be negative since Z points down
		tmp = (ui->thr_stick + 1.0)/2.0;
		tmp = tmp * (MAX_Z_COMPONENT - MIN_Z_COMPONENT);
		sp->Z_throttle = -(tmp + MIN_Z_COMPONENT);

		// if throttle stick is down all the way, probably landed, so
		// keep the yaw setpoint at current yaw so it takes off straight
		if(ui->thr_stick < -0.95){
			sp->yaw = cs->yaw;
			sp->yaw_rate = 0.0;
		}
		// otherwise, scale yaw_rate by max yaw rate in rad/s
		// also apply deadzone to prevent drift
		else{
			tmp = apply_deadzone(ui->yaw_stick, YAW_DEADZONE);
			sp->yaw_rate = tmp * MAX_YAW_RATE;
		}
		// done!
		break;

	// test bench limits throttle to
	case TEST_BENCH:
		// turn off all feedback controllers
		sp->en_alt_ctrl = 0;
		sp->en_6dof = 1;
		sp->en_rpy_ctrl = 0;
		// set XYZ thrusts
		tmp = (ui->thr_stick + 1.0)/2.0;
		sp->Z_throttle = tmp *TEST_BENCH_HOVER_THR;
		sp->Y_throttle = ui->roll_stick;
		sp->X_throttle = ui->pitch_stick;
		break;


	default: // should never get here
		fprintf(stderr,"ERROR in setpoint_manager thread, unknown flight mode\n");
		break;

	} // end switch(ui->flight_mode)
}





int setpoint_manager_cleanup()
{
	initialized=0;
	memset(sp,0,sizeof(setpoint_t));
	return 0;
}
