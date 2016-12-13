/*******************************************************************************
* setpoint_manager.c
*
* This serves to allow the feedback controller to be as simple and clean as 
* possible by putting all high-level manipulation of the setpoints here.
* Then feedback-controller only needs to march the filters and zero them out
* when arming or enabling controllers
*******************************************************************************/
#define _GNU_SOURCE
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <roboticscape.h>
#include "fly_function_declarations.h"
#include "fly_defs.h"
#include "fly_types.h"

setpoint_t*		sp; // pointer to external setpoint_t struct 
user_input_t*	ui; // pointer to external user_input_t struct
cstate_t*		cs;
fly_settings_t* set;
pthread_t setpoint_manager_thread;



/************************************************************************
*	wait_for_arming_sequence()
*	
*	blocking_function that returns after the user has released the
*	kill_switch and toggled the throttle stick up and down
************************************************************************/
int wait_for_arming_sequence(){
START:
	// wait for level MAV before starting
	while(fabs(cs->roll)>ARM_TIP_THRESHOLD||fabs(cs->pitch)>ARM_TIP_THRESHOLD){
		usleep(100000);
		if(rc_get_state()==EXITING) return 0;
	}
	// wait for kill switch to be switched to ARMED
	while(ui->kill_switch==DISARMED){ 
		usleep(100000);
		if(rc_get_state()==EXITING) return 0;
	}
	//wait for throttle down
	while(ui->thr_stick > -0.9){       
		usleep(100000);
		if(rc_get_state()==EXITING) return 0;
		if(ui->kill_switch==DISARMED) goto START;
	}
	//wait for throttle up
	while(ui->thr_stick < 0.9){ 
		usleep(100000);
		if(rc_get_state()==EXITING) return 0;
		if(ui->kill_switch==DISARMED) goto START;
	}
	//wait for throttle down
	while(ui->thr_stick > -0.9){ 
		usleep(100000);
		if(rc_get_state()==EXITING) return 0;
		if(ui->kill_switch==DISARMED) goto START;
	}
	// final check of kill switch and level before arming
	if(ui->kill_switch==DISARMED) goto START;
	if(fabs(cs->roll)>ARM_TIP_THRESHOLD||fabs(cs->pitch)>ARM_TIP_THRESHOLD){
		goto START;
	}

	// return, ready to arm
	return 0;
}


/*******************************************************************************
* void* setpoint_manager(void* ptr)
*
* 
*******************************************************************************/
void* setpoint_manager(void* ptr){
	int i;
	float tmp;

	// wait for IMU to settle
	disarm_controller();
	usleep(1000000);
	usleep(1000000);
	rc_set_led(RED,0);
	rc_set_led(GREEN,1);
	
	// run until state indicates thread should close
	while(rc_get_state()!=EXITING){

		// // record previous flight mode to detect changes
		// previous_flight_mode = user_interface.flight_mode; 
		// run about as fast as the core itself 
		usleep(1000000/SETPOINT_MANAGER_HZ); 

		// if PAUSED or UNINITIALIZED, go back to waiting
		if(rc_get_state()!=RUNNING) continue;
	
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
			tmp = (ui->thr_stick + 1.0)/2.0;
			tmp = tmp * (MAX_Z_THROTTLE - MIN_Z_THROTTLE);
			sp->Z_throttle = tmp + MIN_Z_THROTTLE;
			
			// scale yaw_rate by max yaw rate in rad/s
			// also apply deadzone to prevent drift
			tmp = apply_deadzone(ui->yaw_stick, YAW_DEADZONE);
			sp->yaw_rate = tmp * MAX_YAW_RATE;
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
			tmp = (ui->thr_stick + 1.0)/2.0;
			tmp = tmp * (MAX_Z_THROTTLE - MIN_Z_THROTTLE);
			sp->Z_throttle = tmp + MIN_Z_THROTTLE;

			// scale yaw_rate by max yaw rate in rad/s
			// also apply deadzone to prevent drift
			tmp = apply_deadzone(ui->yaw_stick, YAW_DEADZONE);
			sp->yaw_rate = tmp * MAX_YAW_RATE;
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
			printf("ERROR: unknown flight mode\n");
			break;
		
		} // end switch(ui->flight_mode)
	} // end while(rc_get_state()!=EXITING)

	disarm_controller();
	return NULL;
}


/*******************************************************************************
* int start_setpoint_manager(setpoint_t* setpoint, user_input_t* user_input, \
							cstate_t* cstate, fly_settings_t* settings)
* Starts the setpoint manager thread
*******************************************************************************/
int start_setpoint_manager(setpoint_t* setpoint, user_input_t* user_input, \
							cstate_t* cstate, fly_settings_t* settings){
	sp = setpoint;
	ui = user_input;
	cs = cstate;
	set = settings;

	struct sched_param params = {SETPOINT_MANAGER_PRIORITY};
	pthread_setschedparam(setpoint_manager_thread, SCHED_FIFO, &params);
	pthread_create(&setpoint_manager_thread, NULL, &setpoint_manager, NULL);
	return 0;
}

/*******************************************************************************
* int join_setpoint_manager_thread()
*
* Waits for the setpoint manager thread to exit. Returns 0 if the thread exited 
* cleanly. Returns -1 if the exit timed out.
*******************************************************************************/
int join_setpoint_manager_thread(){
	// wait for the thread to exit
	struct timespec timeout;
	clock_gettime(CLOCK_REALTIME, &timeout);
	timespec_add(&timeout, SETPOINT_MANAGER_TIMEOUT);
	int thread_err = 0;
	thread_err = pthread_timedjoin_np(setpoint_manager_thread, NULL, &timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: input_manager_thread exit timeout\n");
		return -1;
	}
	return 0;
}