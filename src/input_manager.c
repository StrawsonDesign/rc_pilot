/**
 * @file input_manager.c
 */

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <math.h> // for fabs

#include <rc/start_stop.h>
#include <rc/pthread.h>
#include <rc/time.h>
#include <rc/dsm.h>
#include <rc/math/other.h>

#include <input_manager.h>
#include <settings.h>
#include <rc_pilot_defs.h>
#include <thread_defs.h>

user_input_t user_input; // extern variable in input_manager.h

static pthread_t input_manager_thread;
static arm_state_t kill_switch = DISARMED; // raw kill switch on the radio


/**
* float apply_deadzone(float in, float zone)
*
* Applies a dead zone to an input stick in. in is supposed to range from -1 to 1
* the dead zone is centered around 0. zone specifies the distance from 0 the
* zone extends.
**/
static double __deadzone(double in, double zone)
{
	if(zone<=0.0) return in;
	if(fabs(in)<=zone) return 0.0;
	if(in>0.0)	return ((in-zone)/(1.0-zone));
	else		return ((in+zone)/(1.0-zone));
}


/**
 * @brief      blocking function that returns after arming sequence is complete
 *
 * @return     0 if successful or already armed, -1 if exiting or problem
 */
static int __wait_for_arming_sequence()
{
	// already armed, just return. Should never do this in normal operation though
	if(user_input.requested_arm_mode == ARMED) return 0;

ARM_SEQUENCE_START:
	// wait for feedback controller to have started
	while(fstate.initialized==0){
		rc_usleep(100000);
		if(rc_get_state()==EXITING) return 0;
	}
	// wait for level
	while(fabs(fstate.roll)>ARM_TIP_THRESHOLD||fabs(fstate.pitch)>ARM_TIP_THRESHOLD){
		rc_usleep(100000);
		if(rc_get_state()==EXITING) return 0;
	}
	// wait for kill switch to be switched to ARMED
	while(kill_switch==DISARMED){
		rc_usleep(100000);
		if(rc_get_state()==EXITING) return 0;
	}
	// wait for throttle up
	while(rc_dsm_ch_normalized(settings.dsm_thr_ch)*settings.dsm_thr_pol < 0.9){
		rc_usleep(100000);
		if(rc_get_state()==EXITING) return 0;
		if(kill_switch==DISARMED) goto ARM_SEQUENCE_START;
	}
	// wait for throttle down
	while(rc_dsm_ch_normalized(settings.dsm_thr_ch)*settings.dsm_thr_pol > -0.9){
		rc_usleep(100000);
		if(rc_get_state()==EXITING) return 0;
		if(kill_switch==DISARMED) goto ARM_SEQUENCE_START;
	}

	// final check of kill switch and level before arming
	if(kill_switch==DISARMED) goto ARM_SEQUENCE_START;
	if(fabs(fstate.roll)>ARM_TIP_THRESHOLD||fabs(fstate.pitch)>ARM_TIP_THRESHOLD){
		goto ARM_SEQUENCE_START;
	}
	return 0;
}


void new_dsm_data_callback()
{
	double new_thr, new_roll, new_pitch, new_yaw, new_mode, new_kill;

	// Read normalized (+-1) inputs from RC radio stick and multiply by
	// polarity setting so positive stick means positive setpoint
	new_thr   = rc_dsm_ch_normalized(settings.dsm_thr_ch)*settings.dsm_thr_pol;
	new_roll  = rc_dsm_ch_normalized(settings.dsm_roll_ch)*settings.dsm_roll_pol;
	new_pitch = rc_dsm_ch_normalized(settings.dsm_pitch_ch)*settings.dsm_pitch_pol;
	new_yaw   = __deadzone(rc_dsm_ch_normalized(settings.dsm_yaw_ch)*settings.dsm_yaw_pol, YAW_DEADZONE);
	new_mode  = rc_dsm_ch_normalized(settings.dsm_mode_ch)*settings.dsm_mode_pol;


	// kill mode behaviors
	switch(settings.dsm_kill_mode){
	case DSM_KILL_DEDICATED_SWITCH:
		new_kill  = rc_dsm_ch_normalized(settings.dsm_kill_ch)*settings.dsm_kill_pol;
		// determine the kill state
		if(new_kill<=0.1){
			kill_switch = DISARMED;
			user_input.requested_arm_mode=DISARMED;
		}
		else{
			kill_switch = ARMED;
		}
		break;

	case DSM_KILL_NEGATIVE_THROTTLE:
		if(new_thr<=-1.1){
			kill_switch = DISARMED;
			user_input.requested_arm_mode=DISARMED;
		}
		else	kill_switch = ARMED;
		break;

	default:
		fprintf(stderr, "ERROR in input manager, unhandled settings.dsm_kill_mode\n");
		return;
	}

	// saturate the sticks to avoid possible erratic behavior
	// throttle can drop below -1 so extend the range for thr
	rc_saturate_double(&new_thr,   -1.0, 1.0);
	rc_saturate_double(&new_roll,  -1.0, 1.0);
	rc_saturate_double(&new_pitch, -1.0, 1.0);
	rc_saturate_double(&new_yaw,   -1.0, 1.0);

	// pick flight mode
	switch(settings.num_dsm_modes){
	case 1:
		user_input.flight_mode = settings.flight_mode_1;
		break;
	case 2:
		// switch will either range from -1 to 1 or 0 to 1.
		// in either case it's safe to use +0.5 as the cutoff
		if(new_mode>0.5) user_input.flight_mode = settings.flight_mode_2;
		else user_input.flight_mode = settings.flight_mode_1;
		break;
	case 3:
		// 3-position switch will have the positions -1, 0, 1 when
		// calibrated correctly. checking +- 0.5 is a safe cutoff
		if(new_mode>0.5) user_input.flight_mode = settings.flight_mode_3;
		else if(new_mode<-0.5) user_input.flight_mode = settings.flight_mode_1;
		else user_input.flight_mode = settings.flight_mode_2;
		break;
	default:
		fprintf(stderr,"ERROR, in input_manager, num_dsm_modes must be 1,2 or 3\n");
		fprintf(stderr,"selecting flight mode 1\n");
		user_input.flight_mode = settings.flight_mode_1;
		break;
	}

	// fill in sticks
	if(user_input.requested_arm_mode==ARMED){
		user_input.thr_stick   = new_thr;
		user_input.roll_stick  = new_roll;
		user_input.pitch_stick = new_pitch;
		user_input.yaw_stick   = new_yaw;
		user_input.requested_arm_mode = kill_switch;
	}
	else{
		// during arming sequence keep sticks zeroed
		user_input.thr_stick   = 0.0;
		user_input.roll_stick  = 0.0;
		user_input.pitch_stick = 0.0;
		user_input.yaw_stick   = 0.0;
	}

	if(user_input.input_active==0){
		user_input.input_active=1; // flag that connection has come back online
		printf("DSM CONNECTION ESTABLISHED\n");
	}
	return;

}


void dsm_disconnect_callback(void)
{
	user_input.thr_stick = 0.0;
	user_input.roll_stick = 0.0;
	user_input.pitch_stick = 0.0;
	user_input.yaw_stick = 0.0;
	user_input.input_active = 0;
	kill_switch = DISARMED;
	user_input.requested_arm_mode=DISARMED;
	fprintf(stderr, "LOST DSM CONNECTION\n");
}


void* input_manager(void* ptr)
{
	user_input.initialized = 1;
	// wait for first packet
	while(rc_get_state()!=EXITING){
		if(user_input.input_active) break;
		rc_usleep(1000000/INPUT_MANAGER_HZ);
	}

	// not much to do since the DSM callbacks to most of it. Later some
	// logic to handle other inputs such as mavlink/bluetooth/wifi
	while(rc_get_state()!=EXITING){
		// if the core got disarmed, wait for arming sequence
		if(user_input.requested_arm_mode == DISARMED){
			__wait_for_arming_sequence();
			// user may have pressed the pause button or shut down while waiting
			// check before continuing
			if(rc_get_state()!=RUNNING) continue;
			else{
				user_input.requested_arm_mode=ARMED;
				//printf("\n\nDSM ARM REQUEST\n\n");
			}
		}
		// wait
		rc_usleep(1000000/INPUT_MANAGER_HZ);
	}
	return NULL;
}



int input_manager_init()
{
	user_input.initialized = 0;
	int i;
	// start dsm hardware
	if(rc_dsm_init()==-1){
		fprintf(stderr, "ERROR in input_manager_init, failed to initialize dsm\n");
		return -1;
	}
	rc_dsm_set_disconnect_callback(dsm_disconnect_callback);
	rc_dsm_set_callback(new_dsm_data_callback);
	// start thread
	if(rc_pthread_create(&input_manager_thread, &input_manager, NULL,
				SCHED_FIFO, INPUT_MANAGER_PRI)==-1){
		fprintf(stderr, "ERROR in input_manager_init, failed to start thread\n");
		return -1;
	}
	// wait for thread to start
	for(i=0;i<50;i++){
		if(user_input.initialized) return 0;
		rc_usleep(50000);
	}
	fprintf(stderr, "ERROR in input_manager_init, timeout waiting for thread to start\n");
	return -1;
}


int input_manager_cleanup()
{
	if(user_input.initialized==0){
		fprintf(stderr, "WARNING in input_manager_cleanup, was never initialized\n");
		return -1;
	}
	// wait for the thread to exit
	if(rc_pthread_timed_join(input_manager_thread, NULL, INPUT_MANAGER_TOUT)==1){
		fprintf(stderr,"WARNING: in input_manager_cleanup, thread join timeout\n");
		return -1;
	}
	// stop dsm
	rc_dsm_cleanup();
	return 0;
}
