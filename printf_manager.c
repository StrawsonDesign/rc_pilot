/*******************************************************************************
* printf_manager.c
*
*******************************************************************************/
#define _GNU_SOURCE
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <roboticscape.h>
#include "fly_defs.h"
#include "fly_types.h"

cstate_t*		cs;		// pointer to external core_state_t struct 
setpoint_t*		sp;		// pointer to external setpoint struct
user_input_t*	ui;
fly_settings_t*	set;

pthread_t printf_manager_thread;

/************************************************************************
* int print_flight_mode(flight_mode_t mode)
*
* print a flight mode to console
************************************************************************/
int print_flight_mode(flight_mode_t mode){
	switch(mode){
	case DIRECT_THROTTLE:
		printf("DIRECT_THROTTLE ");
		break;
	case FALLBACK_4DOF:
		printf("FALLBACK_4DOF  ");
		break;
	case TEST_BENCH:
		printf("TEST_BENCH     ");
		break;
	default:
		printf("ERROR: unknown flight mode\n"); 
		break;
	}
	return 0;
}

/************************************************************************
* int print_header()
*
* prints row of names to label the numbers being printed
************************************************************************/
int print_header(){
	printf("\n");
	if(set->printf_arm){
		printf("  arm   |");
	}
	if(set->printf_altitude){
		printf(" alt(m) |");
	}
	if(set->printf_rpy){
		printf("roll |pitch| yaw |");
	}
	if(set->printf_sticks){
		printf(" thr |roll |pitch| yaw |");
	}
	if(set->printf_setpoint){
		printf("sp_alt |sp_roll|sp_ptch|sp_yaw |");
	}
	if(set->printf_u){
		printf(" U0X | U1Y | U2Z | U3r | U4p | U5y |");
	}
	if(set->printf_motors){
		printf(" M1 | M2 | M3 | M4 | M5 | M6 |");
	}
	if(set->printf_mode){
		printf("   MODE ");
	}

	printf("\n");
	fflush(stdout);
	return 0;
}


/*******************************************************************************
* void* printf_manager(void* ptr)
*
*******************************************************************************/
void* printf_manager(void* ptr){
	arm_state_t prev_arm_state;

	print_header();
	prev_arm_state = sp->arm_state;

	while(rc_get_state()==EXITING){

		// re-print header on disarming
		if(sp->arm_state==DISARMED && prev_arm_state==ARMED){
			print_header();
		}

		printf("\r"); 
		if(set->printf_arm){
			if(sp->arm_state == ARMED) printf(" ARMED  |");
			else                       printf("DISARMED|");
		}
		if(set->printf_altitude){
			printf("%7.2f |", cs->altitude);
		}
		if(set->printf_rpy){
			printf("%6.2f |", cs->roll);
			printf("%6.2f |", cs->roll);
			printf("%6.2f |", cs->roll);
		}
		if(set->printf_sticks){
			printf("%6.2f |", ui->thr_stick);
			printf("%6.2f |", ui->roll_stick);
			printf("%6.2f |", ui->pitch_stick);
			printf("%6.2f |", ui->yaw_stick);
		}
		if(set->printf_setpoint){
			printf("%6.2f |", sp->altitude);
			printf("%6.2f |", sp->roll);
			printf("%6.2f |", sp->pitch);
			printf("%6.2f |", sp->yaw);
		}
		if(set->printf_u){
			printf("%6.2f |", cs->u[0]);
			printf("%6.2f |", cs->u[1]);
			printf("%6.2f |", cs->u[2]);
			printf("%6.2f |", cs->u[3]);
			printf("%6.2f |", cs->u[4]);
			printf("%6.2f |", cs->u[5]);
		}
		if(set->printf_motors){
			printf("%5.2f |", cs->m[0]);
			printf("%5.2f |", cs->m[1]);
			printf("%5.2f |", cs->m[2]);
			printf("%5.2f |", cs->m[3]);
			printf("%5.2f |", cs->m[4]);
			printf("%5.2f |", cs->m[5]);
		}
		if(set->printf_mode){
			print_flight_mode(ui->flight_mode);
		}

		fflush(stdout);
		prev_arm_state = sp->arm_state;
		usleep(1000000/PRINTF_MANAGER_HZ);
	}
	return NULL;
}


/*******************************************************************************
* int start_printf_manager(cstate_t* cstate, setpoint_t* setpoint, \
			user_input_t* user_input, fly_settings_t* settings)
*
* Start the printf_manager which should be the only thing printing to the screen
* besides error messages from other threads.
*******************************************************************************/
int start_printf_manager(cstate_t* cstate, setpoint_t* setpoint, \
			user_input_t* user_input, fly_settings_t* settings){
	cs = cstate;
	sp = setpoint;
	ui = user_input;
	set = settings;
	pthread_create(&printf_manager_thread, NULL, &printf_manager, NULL);
	struct sched_param params = {PRINTF_MANAGER_PRIORITY};
	pthread_setschedparam(printf_manager_thread, SCHED_FIFO, &params);
	usleep(1000);
	return 0;
}

/*******************************************************************************
* int join_printf_manager_thread()
*
* Waits for the printf_manager thread to exit. Returns 0 if the thread exited 
* cleanly. Returns -1 if the exit timed out.
*******************************************************************************/
int join_printf_manager_thread(){
	// wait for the thread to exit
	struct timespec timeout;
	clock_gettime(CLOCK_REALTIME, &timeout);
	rc_timespec_add(&timeout, PRINTF_MANAGER_TIMEOUT);
	int thread_err = 0;
	thread_err = pthread_timedjoin_np(printf_manager_thread, NULL, &timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: printf_manager_thread exit timeout\n");
		return -1;
	}
	return 0;
}
