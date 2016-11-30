/*******************************************************************************
* setpoint_manager.c
*
*******************************************************************************/

#include <useful_inlcudes.h>
#include <robotics_cape.h>
#include "fly_defs.h"
#include "fly_types.h"

core_state_t* core_state; // pointer to external core_state_t struct 
fly_settings_t* settings; // pointer to external settings struct
pthread_t battery_manager_thread;
d_filter_t batt_filter;

/*******************************************************************************
* void* battery_manager(void* ptr)
*
* 
*******************************************************************************/
void* battery_manager(void* ptr){
	float new_v;
	while(get_state()!=EXITING){
		switch(settings->battery_connection){
		case BALANCE_PLUG:
			new_v = get_batt_voltage();
			break;
		case DC_BARREL_JACK:
			new_v = get_jack_voltage();
			break;
		default:
			printf("ERROR: battery_connection mode not implemented in battery_manager\n");
			return NULL;
		}
		
		// if battery is below a threshold consider disconnected
		if(new_v<2) new_v = 0.0;
		core_state->v_batt = march_filter(&batt_filter, new_v);
		usleep(1000000 / BATTERY_MANAGER_HZ);
	}
	return NULL;
}


/*******************************************************************************
* int start_battery_manager_thread(core_state_t* cs, fly_settings_t set*)
*
* 
*******************************************************************************/
int start_battery_manager_thread(core_state_t* cs, fly_settings_t set*){
	core_state = cs;
	settings = set;
	batt_filter = create_moving_average_filter(5);
	struct sched_param params = {BATTERY_MANAGER_PRIORITY};
	pthread_setschedparam(battery_manager_thread, SCHED_FIFO, &params);
	pthread_create(&battery_manager_thread, NULL, &battery_manager, NULL);
	usleep(1000);
	return 0;
}

/*******************************************************************************
* int join_battery_manager_thread()
*
* Waits for the battery manager thread to exit. Returns 0 if the thread exited 
* cleanly. Returns -1 if the exit timed out.
* This should only be called after the program flow state is set to EXITING as 
* that's the only thing that will cause the thread to exit on its own safely.
*******************************************************************************/
int join_battery_manager_thread(){
	// wait for the thread to exit
	struct timespec timeout;
	clock_gettime(CLOCK_REALTIME, &timeout);
	timespec_add(&timeout, BATTERY_MANAGER_TIMEOUT);
	int thread_err = 0;
	thread_err = pthread_timedjoin_np(battery_manager_thread, NULL, &timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: battery_manager_thread exit timeout\n");
		return -1;
	}
	destroy_filter(&battery_filter);
	return 0;
}