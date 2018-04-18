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

#include <fly_types.h>
#include <fly_defs.h>

static fly_settings_t* set;
static user_input_t* ui; // pointer to external user_input_t struct
static pthread_t input_manager_thread;
static int dsm_connect_flag = 0; // set to 1 every time packet received
static int dsm_disconnect_flag = 0;
static int last_disconnect_flag = 0;

void new_dsm_data_callback()
{
	float new_thr, new_roll, new_pitch, new_yaw, new_mode, new_kill;

	// Read normalized (+-1) inputs from RC radio stick and multiply by
	// polarity setting so positive stick means positive setpoint
	new_thr   = rc_dsm_ch_normalized(set->dsm_thr_ch)*set->dsm_thr_pol;
	new_roll  = rc_dsm_ch_normalized(set->dsm_roll_ch)*set->dsm_roll_pol;
	new_pitch = rc_dsm_ch_normalized(set->dsm_pitch_ch)*set->dsm_pitch_pol;
	new_yaw   = rc_dsm_ch_normalized(set->dsm_yaw_ch)*set->dsm_yaw_pol;
	new_kill  = rc_dsm_ch_normalized(set->dsm_kill_ch)*set->dsm_kill_pol;
	new_mode  = rc_dsm_ch_normalized(set->dsm_mode_ch)*set->dsm_mode_pol;

	// saturate the sticks to avoid possible erratic behavior
	// throttle can drop below -1 so extend the range for thr
	rc_saturate_float(&new_thr,   -1.5, 1.0);
	rc_saturate_float(&new_roll,  -1.0, 1.0);
	rc_saturate_float(&new_pitch, -1.0, 1.0);
	rc_saturate_float(&new_yaw,   -1.0, 1.0);

	// fill in sticks
	ui->thr_stick   = new_thr;
	ui->roll_stick  = new_roll;
	ui->pitch_stick = new_pitch;
	ui->yaw_stick   = new_yaw;

	// determine the kill state
	if(new_kill<=0) ui->kill_switch = DISARMED;
	else		ui->kill_switch = ARMED;

	// pick flight mode
	switch(set->num_dsm_modes){
	case 1:
		ui-> flight_mode = set->flight_mode_1;
		break;
	case 2:
		if(new_mode>0) ui-> flight_mode = set->flight_mode_2;
		else ui-> flight_mode = set->flight_mode_1;
		break;
	case 3:
		if(new_mode>0.5) ui-> flight_mode = set->flight_mode_3;
		else if(new_mode<-0.5) ui-> flight_mode = set->flight_mode_1;
		else ui-> flight_mode = set->flight_mode_2;
		break;
	default:
		fprintf(stderr,"ERROR, num_dsm_modes must be 1,2 or 3\n");
		ui-> flight_mode = set->flight_mode_1;
		break;
	}
	ui->user_input_active = 1;

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
	// wait for first packet
	while(rc_get_state()!=EXITING){
		if(dsm_connect_flag) break;
		rc_usleep(1000000/INPUT_MANAGER_HZ);
	}

	// monitor for disconnects
	while(rc_get_state()!=EXITING){
		// if dsm is not active, keep the ui zero'd out
		if(dsm_disconnect_flag==1){
			ui->user_input_active = 0;
			ui->thr_stick = 0;
			ui->roll_stick = 0;
			ui->pitch_stick = 0;
			ui->yaw_stick = 0;
			ui->kill_switch = DISARMED;
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


int start_input_manager(user_input_t* user_input, fly_settings_t* settings)
{
	// Sanity Checks
	if(user_input==NULL){
		fprintf(stderr,"ERROR: in start input_manager, user_input struct pointer NULL\n");
		return 0;
	}
	if(settings==NULL){
		fprintf(stderr,"ERROR: in start input_manager, settings struct pointer NULL\n");
		return 0;
	}
	// make local copy of remote variables
	ui = user_input;
	set = settings;
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
	rc_usleep(1000);
	return 0;
}


int join_input_manager_thread()
{
	// wait for the thread to exit
	if(rc_pthread_timed_join(input_manager_thread, NULL, INPUT_MANAGER_TOUT)==1){
		fprintf(stderr,"WARNING: input_manager_thread exit timeout\n");
		return -1;
	}
	// stop dsm
	rc_dsm_cleanup();
	return 0;
}
