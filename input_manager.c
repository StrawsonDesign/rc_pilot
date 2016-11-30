/*******************************************************************************
* input_manager.c
*
* functions to start and stop the input manager thread which watches for new 
* dsm data and translate into local user mode
*******************************************************************************/

#include <useful_inlcudes.h>
#include <robotics_cape.h>
#include "fly_defs.h"
#include "fly_types.h"

user_input_t* ui; // pointer to external user_input_t struct 
fly_settings_t* set;
pthread_t input_manager_thread;

/*******************************************************************************
* void* input_manager(void* ptr)
*
* Watch for new DSM data and translate into local user mode
*******************************************************************************/
void* input_manager(void* ptr){
	
	float new_thr, new_roll, new_pitch, new_yaw, new_kill;

	if(ui==NULL){
		printf("ERROR: can't start input_manager, ui struct pointer NULL\n");
		return -1;
	}
	
	while(get_state()!=EXITING){
		
		usleep(1000000/INPUT_MANAGER_HZ);

		// if dsm is not active, keep the ui zero'd out
		if(!is_dsm_active()){
			ui->user_input_active = 0.0;
			ui->throttle_stick = 0.0;
			ui->roll_stick = 0.0;
			ui->pitch_stick = 0.0;
			ui->yaw_stick = 0.0;
			ui->kill_switch = DISARMED;
			continue;
		}

		// if dsm is connected but no new data is available, just wait and
		// check next round
		if(!is_new_dsm_data()) continue;

		// Read normalized (+-1) inputs from RC radio stick and multiply by 
		// polarity setting so positive stick means positive setpoint
		new_thr   = get_dsm_ch_normalized(set->dsm_throttle_ch)*set->dsm_throttle_pol;
		new_roll  = get_dsm_ch_normalized(set->dsm_roll_ch)*set->dsm_roll_pol;
		new_pitch = get_dsm_ch_normalized(set->dsm_pitch_ch)*set->dsm_pitch_pol;
		new_yaw   = get_dsm_ch_normalized(set->dsm_yaw_ch)*set->dsm_yaw_pol;
		new_mode  = get_dsm_ch_normalized(set->dsm_mode_ch)*set->mode_pol;
		new_kill  = get_dsm_ch_normalized(set->dsm_kill_ch)*set->dsm_kill_pol;
		
		// saturate the sticks to avoid possible erratic behavior
		saturate_float(&new_thr,   -1.0, 1.0);
		saturate_float(&new_roll,  -1.0, 1.0);
		saturate_float(&new_pitch, -1.0, 1.0);
		saturate_float(&new_yaw,   -1.0, 1.0);
		
		// fill in sticks
		ui->thr_stick   = new_thr;
		ui->roll_stick  = new_roll;
		ui->pitch_stick = new_pitch;
		ui->yaw_stick   = new_yaw;

		// determine the kill state
		if(new_kill<=0) ui->kill_switch = DISARMED;
		else 			ui->kill_switch = ARMED;
		
		// pick flight mode
		if(set->dsm_num_modes == 3){
			if		(new_mode<-0.5)	ui->flight_mode = set->flight_mode_1;
			else if	(new_mode> 0.5)	ui->flight_mode = set->flight_mode_2;
			else					ui->flight_mode = set->flight_mode_3;
		}
		else if (set->dsm_num_modes == 2){
			if	(new_mode > 0.0)	ui->flight_mode = set->flight_mode_2;
			else					ui->flight_mode = set->flight_mode_1;
		}
		else if (set->dsm_num_modes == 1){
									ui->flight_mode = set->flight_mode_1;
		}
		else{
			printf("ERROR: set->dsm_num_modes can only be 1,2 or 3");
			ui->kill_switch = DISARMED;
		}
	}
}


/*******************************************************************************
* int start_input_manager(user_input_t* user_input, fly_settings* settings)
*
* Watch for new dsm data and translate into local user mode
*******************************************************************************/
int start_input_manager(user_input_t* user_input, fly_settings* settings){
	ui = user_input;
	set = settings;
	struct sched_param params = {INPUT_MANAGER_PRIORITY};
	pthread_setschedparam(input_manager_thread, SCHED_FIFO, &params);
	pthread_create(&input_manager_thread, NULL, &input_manager, NULL);
	usleep(1000);
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
	timespec_add(&timeout, INPUT_MANAGER_TIMEOUT);
	int thread_err = 0;
	thread_err = pthread_timedjoin_np(input_manager_thread, NULL, &timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: input_manager_thread exit timeout\n");
		return -1;
	}
	return 0;
}