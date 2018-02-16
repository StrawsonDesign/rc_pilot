/**
 * @file battery_manager.c
 */


#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include <rc/state.h>
#include <rc/time.h>
#include <rc/battery.h>
#include <rc/pthread_helpers.h>

#include <fly_defs.h>
#include <battery_manager.h>

fly_settings_t* set;	// pointer to external settings struct
cstate_t* cs;		// pointer to external core_state_t struct
pthread_t pthread;


void* battery_manager_func(void* ptr){
	float new_v;
	while(rc_get_state()!=EXITING){
		if(set->battery_connection==DC_BARREL_JACK){
			new_v = rc_dc_jack_voltage();
		}
		else if(set->battery_connection==BALANCE_PLUG){
			new_v = rc_battery_voltage();
		}
		else{
			fprintf(stderr,"ERROR: invalid battery_connection_t\n");
			new_v = set->v_nominal;
		}
		cs->v_batt = new_v;
		rc_usleep(1000000 / BATTERY_MANAGER_HZ);
	}
	return NULL;
}



int start_battery_manager(cstate_t* core_state, fly_settings_t* settings){
	cs = core_state;
	set = fly_settings;
	if(rc_pthread_create(&pthread, battery_manager_func, NULL, SCHED_FIFO, BATTERY_MANAGER_PRI)<0){
		fprintf(stderr,"ERROR in start_battery_manager, failed to start thread\n");
		return -1;
	}
	rc_usleep(1000);
	return 0;
}


int join_battery_manager_thread(){
	// wait for the thread to exit
	int ret = rc_pthread_timed_join(pthread,NULL,BATTERY_MANAGER_TOUT);
	if(ret==1) fprintf(stderr,"WARNING: battery_manager_thread exit timeout\n");
	else if(ret==-1) fprintf(stderr,"ERROR: failed to join battery_manager thread\n");
	return ret;
}


