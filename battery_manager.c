/*******************************************************************************
* setpoint_manager.c
*
*******************************************************************************/

#include <roboticscape-usefulincludes.h>
#include <roboticscape.h>
#include "fly_defs.h"
#include "fly_types.h"

fly_settings_t* set;	// pointer to external settings struct
cstate_t* cs;		// pointer to external core_state_t struct 
pthread_t battery_manager_thread;

/*******************************************************************************
* void* battery_manager(void* ptr)
*
* 
*******************************************************************************/
void* battery_manager(void* ptr){
	double new_v;
	while(rc_get_state()!=EXITING){
		if(set->battery_connection==DC_BARREL_JACK){
			new_v = get_dc_jack_voltage();
		}
		else if(set->battery_connection==BALANCE_PLUG){
			new_v = get_battery_voltage();
		}
		else{
			printf("ERROR: invalid battery_connection_t\n");
			new_v = set->v_nominal;
		}
		cs->v_batt = new_v;
		usleep(1000000 / BATTERY_MANAGER_HZ);
	}
	return NULL;
}


/*******************************************************************************
* int start_battery_manager(cstate_t* core_state, fly_settings_t* fly_settings)
*
* 
*******************************************************************************/
int start_battery_manager(cstate_t* core_state, fly_settings_t* fly_settings){
	cs = core_state;
	set = fly_settings;
	pthread_create(&battery_manager_thread, NULL, &battery_manager, NULL);
	struct sched_param params = {BATTERY_MANAGER_PRIORITY};
	pthread_setschedparam(battery_manager_thread, SCHED_FIFO, &params);
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
	return 0;
}