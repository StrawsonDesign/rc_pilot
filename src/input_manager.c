/**
 * @file input_manager.c
 */

#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include <rc/start_stop.h>
#include <rc/pthread.h>
#include <rc/time.h>
#include <rc/dsm.h>
#include <rc/math/other.h>

#include <input_manager.h>
#include <settings.h>
#include <thread_defs.h>

static user_input_t ui; // current user_input_t struct

static pthread_t input_manager_thread;
static int dsm_connect_flag = 0; // set to 1 every time packet received
static int dsm_disconnect_flag = 0;
static int last_disconnect_flag = 0;
static int started_flag = 0;

void new_dsm_data_callback()
{
	float new_thr, new_roll, new_pitch, new_yaw, new_mode, new_kill;

	// Read normalized (+-1) inputs from RC radio stick and multiply by
	// polarity setting so positive stick means positive setpoint
	new_thr   = rc_dsm_ch_normalized(settings.dsm_thr_ch)*settings.dsm_thr_pol;
	new_roll  = rc_dsm_ch_normalized(settings.dsm_roll_ch)*settings.dsm_roll_pol;
	new_pitch = rc_dsm_ch_normalized(settings.dsm_pitch_ch)*settings.dsm_pitch_pol;
	new_yaw   = rc_dsm_ch_normalized(settings.dsm_yaw_ch)*settings.dsm_yaw_pol;
	new_kill  = rc_dsm_ch_normalized(settings.dsm_kill_ch)*settings.dsm_kill_pol;
	new_mode  = rc_dsm_ch_normalized(settings.dsm_mode_ch)*settings.dsm_mode_pol;

	// saturate the sticks to avoid possible erratic behavior
	// throttle can drop below -1 so extend the range for thr
	rc_saturate_float(&new_thr,   -1.5, 1.0);
	rc_saturate_float(&new_roll,  -1.0, 1.0);
	rc_saturate_float(&new_pitch, -1.0, 1.0);
	rc_saturate_float(&new_yaw,   -1.0, 1.0);

	// fill in sticks
	ui.thr_stick   = new_thr;
	ui.roll_stick  = new_roll;
	ui.pitch_stick = new_pitch;
	ui.yaw_stick   = new_yaw;

	// determine the kill state
	if(new_kill<=0) ui.kill_switch = DISARMED;
	else		ui.kill_switch = ARMED;

	// pick flight mode
	switch(settings.num_dsm_modes){
	case 1:
		ui.flight_mode = settings.flight_mode_1;
		break;
	case 2:
		if(new_mode>0.0f) ui.flight_mode = settings.flight_mode_2;
		else ui.flight_mode = settings.flight_mode_1;
		break;
	case 3:
		if(new_mode>0.5f) ui.flight_mode = settings.flight_mode_3;
		else if(new_mode<-0.5f) ui.flight_mode = settings.flight_mode_1;
		else ui.flight_mode = settings.flight_mode_2;
		break;
	default:
		fprintf(stderr,"ERROR, in input_manager, num_dsm_modes must be 1,2 or 3\n");
		ui.flight_mode = settings.flight_mode_1;
		break;
	}
	ui.user_input_active = 1;

	if(dsm_connect_flag==0){
		printf("DSM CONNECTION ESTABLISHED\n");
		dsm_connect_flag=1; // flag that connection has come back online
	}
	dsm_disconnect_flag=0;
	last_disconnect_flag=0;

	return;

}

void* input_manager(void* ptr)
{
	started_flag = 1;
	// wait for first packet
	while(rc_get_state()!=EXITING){
		if(dsm_connect_flag) break;
		rc_usleep(1000000/INPUT_MANAGER_HZ);
	}

	// monitor for disconnects
	while(rc_get_state()!=EXITING){
		// if dsm is not active, keep the ui zero'd out
		if(dsm_disconnect_flag==1){
			ui.user_input_active = 0;
			ui.thr_stick = 0;
			ui.roll_stick = 0;
			ui.pitch_stick = 0;
			ui.yaw_stick = 0;
			ui.kill_switch = DISARMED;
			// reset connection flag
			dsm_connect_flag = 0;
			if(last_disconnect_flag==0){
				fprintf(stderr, "LOST DSM CONNECTION\n");
				last_disconnect_flag=1;
			}

		}
		// set flag to see if dsm is coming in
		dsm_disconnect_flag = 1;
		// wait
		rc_usleep(1000000/INPUT_MANAGER_HZ);
	}
	return NULL;
}


int input_manager_init()
{
	int i;
	// start dsm hardware
	if(rc_dsm_init()==-1){
		fprintf(stderr, "ERROR in start_input_manager, failed to initialize dsm\n");
		return -1;
	}
	// start thread
	if(rc_pthread_create(&input_manager_thread, &input_manager, NULL,
				SCHED_FIFO, INPUT_MANAGER_PRI)==-1){
		fprintf(stderr, "ERROR in start_input_manager, failed to start thread\n");
		return -1;
	}
	// wait for thread to start
	for(i=0;i<50;i++){
		if(started_flag) return 0;
		rc_usleep(10000);
	}
	fprintf(stderr, "ERROR in input_manager_init, timeout waiting for thread to start\n");
	return -1;
}


int input_manager_cleanup()
{
	if(started_flag==0){
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
