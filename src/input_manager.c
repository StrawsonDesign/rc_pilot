/*******************************************************************************
* input_manager.c
*
* functions to start and stop the input manager thread which watches for new 
* dsm data and translate into local user mode
*******************************************************************************/

#define  _GNU_SOURCE
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <roboticscape.h>
#include "fly_defs.h"
#include "fly_types.h"

fly_settings_t* set;
user_input_t* ui; // pointer to external user_input_t struct 
pthread_t input_manager_thread;

/*******************************************************************************
* void* input_manager(void* ptr)
*
* Watch for new dsm data and translate into local user mode
*******************************************************************************/
void* input_manager(void* ptr){
	
	float new_thr, new_roll, new_pitch, new_yaw, new_mode, new_kill;

	if(ui==NULL){
		printf("ERROR: can't start input_manager, ui struct pointer NULL\n");
		return NULL;
	}
	
	while(rc_get_state()!=EXITING){
		rc_usleep(1000000/INPUT_MANAGER_HZ);

		// if dsm is not active, keep the ui zero'd out
		if(!rc_is_dsm_active()){
			ui->user_input_active = 0;
			ui->thr_stick = 0;
			ui->roll_stick = 0;
			ui->pitch_stick = 0;
			ui->yaw_stick = 0;
			ui->kill_switch = DISARMED;
			continue;
		}

		// if dsm is connected but no new data is available, just wait and
		// check next round
		if(!rc_is_new_dsm_data()) continue;

		// Read normalized (+-1) inputs from RC radio stick and multiply by 
		// polarity setting so positive stick means positive setpoint
		new_thr   = rc_get_dsm_ch_normalized(set->dsm_thr_ch)*set->dsm_thr_pol;
		new_roll  = rc_get_dsm_ch_normalized(set->dsm_roll_ch)*set->dsm_roll_pol;
		new_pitch = rc_get_dsm_ch_normalized(set->dsm_pitch_ch)*set->dsm_pitch_pol;
		new_yaw   = rc_get_dsm_ch_normalized(set->dsm_yaw_ch)*set->dsm_yaw_pol;
		new_kill  = rc_get_dsm_ch_normalized(set->dsm_kill_ch)*set->dsm_kill_pol;
		new_mode  = rc_get_dsm_ch_normalized(set->dsm_mode_ch)*set->dsm_mode_pol;
		
		// saturate the sticks to avoid possible erratic behavior
		rc_saturate_float(&new_thr,   -1.0, 1.0);
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
		else 			ui->kill_switch = ARMED;
			
		// pick flight mode
		if(set->num_dsm_modes == 1){
			ui-> flight_mode = set->flight_mode_1;
		}
		else if(set->num_dsm_modes == 2){
			if(new_mode>0) ui-> flight_mode = set->flight_mode_2;
			else ui-> flight_mode = set->flight_mode_1;
		}
		else if(set->num_dsm_modes == 3){
			if(new_mode>0.5) ui-> flight_mode = set->flight_mode_3;
			else if(new_mode<-0.5) ui-> flight_mode = set->flight_mode_1;
			else ui-> flight_mode = set->flight_mode_2;
		}
		else{
			printf("ERROR, num_dsm_modes must be 1,2 or 3\n");
			ui-> flight_mode = set->flight_mode_1;
		}
		ui->user_input_active = 1;
	}
	return NULL;
}


/*******************************************************************************
* int start_input_manager(user_input_t* user_input, fly_settings_t* settings)
*
* start the input thread and save pointers to input and settings structs.
*******************************************************************************/
int start_input_manager(user_input_t* user_input, fly_settings_t* settings){
	ui = user_input;
	set = settings;
	pthread_create(&input_manager_thread, NULL, &input_manager, NULL);
	struct sched_param params = {INPUT_MANAGER_PRIORITY};
	pthread_setschedparam(input_manager_thread, SCHED_FIFO, &params);
	rc_usleep(1000);
	return 0;
}

/*******************************************************************************
* int join_input_manager_thread()
*
* Waits for the input manager thread to exit. Returns 0 if the input manager
* thread exited cleanly. Returns -1 if the exit timed out and the thread had
* to be force closed.
* This should only be called after the program flow state is set to EXITING as 
* that's the only thing that will cause the thread to exit on its own safely.
*******************************************************************************/
int join_input_manager_thread(){
	// wait for the thread to exit
	struct timespec timeout;
	clock_gettime(CLOCK_REALTIME, &timeout);
	rc_timespec_add(&timeout, INPUT_MANAGER_TIMEOUT);
	int thread_err = 0;
	thread_err = pthread_timedjoin_np(input_manager_thread, NULL, &timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: input_manager_thread exit timeout\n");
		return -1;
	}
	return 0;
}
