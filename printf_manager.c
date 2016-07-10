/*******************************************************************************
* printf_manager.c
*
*******************************************************************************/

#include <useful_inlcudes.h>
#include <robotics_cape.h>
#include "fly_defs.h"
#include "fly_types.h"

core_state_t* cs; // pointer to external core_state_t struct 
setpoint_t* sp;

pthread_t printf_manager_thread;

/************************************************************************
* int print_flight_mode(flight_mode_t mode)
*
* print a flight mode to console
************************************************************************/
int print_flight_mode(flight_mode_t mode){
	switch(mode){
	case ATTITUDE_DIRECT_THROTTLE:
		printf("ATTITUDE_DIRECT_THROTTLE");
		break;
	case ATTITUDE_ALTITUDE_HOLD:
		printf("ATTITUDE_ALTITUDE_HOLD  ");
		break;
	case 6DOF_CONTROL:
		printf("   6DOF_CONTROL         ");
		break;
	case EMERGENCY_LAND:
		printf("  EMERGENCY_LAND        ");
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
	printf(" alt |");
	printf(" thr |");
	printf("roll |");
	printf(" u_r |");
	printf("pitch|");
	printf(" u_p |");
	printf(" yaw |");
	printf(" u_y |");
	printf("  armed |");
	printf("    mode    ");
	printf("\n");
	fflush(stdout);
	return 0;
}


/*******************************************************************************
* void* printf_manager(void* ptr)
*
*******************************************************************************/
void* printf_manager(void* ptr){
	
	print_header();

	while(get_state()==EXITING){
		printf("\r"); 
		printf("%6.2f ", cs->alt);
		printf("%6.2f ", cs->u[VEC_THR]);
		printf("%6.2f ", cs->roll);
		printf("%6.2f ", cs->u[VEC_ROLL]);
		printf("%6.2f ", cs->pitch);
		printf("%6.2f ", cs->u[VEC_PITCH]);
		printf("%6.2f ", cs->yaw);
		printf("%6.2f ", cs->u[VEC_YAW]);
		
		if(sp->arm_state == ARMED) printf(" ARMED  |");
		else                       printf("DISARMED|");
		
		print_flight_mode();
		fflush(stdout);

		usleep(1000000/PRINTF_MANAGER_HZ);
	}
	return NULL;
}


/*******************************************************************************
* int start_printf_manager(core_state_t* core_state, setpoint_t* setpoint)
*
* Start the printf_manager which should be the only thing printing to the screen
* besides error messages from other threads.
*******************************************************************************/
int start_printf_manager(core_state_t* core_state, setpoint_t* setpoint){
	cs = core_state;
	sp = setpoint;
	struct sched_param params = {PRINTF_MANAGER_PRIORITY};
	pthread_setschedparam(printf_manager_thread, SCHED_FIFO, &params);
	pthread_create(&printf_manager_thread, NULL, &printf_manager, NULL);
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
	timespec_add(&timeout, PRINTF_MANAGER_TIMEOUT);
	int thread_err = 0;
	thread_err = pthread_timedjoin_np(printf_manager_thread, NULL, &timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: printf_manager_thread exit timeout\n");
		return -1;
	}
	return 0;
}