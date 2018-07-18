/**
 * @file printf_manager.c
 */

#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/pthread.h>

#include <printf_manager.h>
#include <input_manager.h>
#include <setpoint_manager.h>
#include <feedback.h>
#include <thread_defs.h>
#include <settings.h>
#include <flight_mode.h>




static pthread_t printf_manager_thread;
static int initialized = 0;

/**
 * @brief      Only used by printf_manager right now, but could be useful elsewhere.
 *
 * @param[in]  mode  The mode
 *
 * @return     0 on success or -1 on error
 */
int print_flight_mode(flight_mode_t mode){
	switch(mode){
	case TEST_BENCH_4DOF:
		printf("%sTEST_BENCH_4DOF     %s",KYEL,KNRM);
		return 0;
	case TEST_BENCH_6DOF:
		printf("TEST_BENCH_6DOF         ");
		return 0;
	case DIRECT_THROTTLE_4DOF:
		printf("%sDIRECT_THROTTLE_4DOF%s",KCYN,KNRM);
		return 0;
	case DIRECT_THROTTLE_6DOF:
		printf("DIRECT_THROTTLE_6DOF    ");
		return 0;
	case ALT_HOLD_4DOF:
		printf("%sALT_HOLD_4DOF       %s",KBLU,KNRM);
		return 0;
	case ALT_HOLD_6DOF:
		printf("ALT_HOLD_6DOF           ");
		return 0;
	default:
		fprintf(stderr,"ERROR in print_flight_mode, unknown flight mode\n");
		return -1;
	}
}


static int __print_header(){
	printf("\n");
	if(settings.printf_arm){
		printf("  arm   |");
	}
	if(settings.printf_altitude){
		printf(" alt(m) |");
	}
	if(settings.printf_rpy){
		printf(" roll|pitch| yaw |");
	}
	if(settings.printf_sticks){
		printf(" thr |roll |pitch| yaw |");
	}
	if(settings.printf_setpoint){
		printf(" sp_a| sp_r| sp_p| sp_y|");
	}
	if(settings.printf_u){
		printf(" U0X | U1Y | U2Z | U3r | U4p | U5y |");
	}
	if(settings.printf_motors){
		printf("  M1 |  M2 |  M3 |  M4 |");
	}
	if(settings.printf_mode){
		printf("   MODE ");
	}

	printf("\n");
	fflush(stdout);
	return 0;
}


static void* __printf_manager_func(__attribute__ ((unused)) void* ptr)
{
	arm_state_t prev_arm_state;

	initialized = 1;
	__print_header();

	prev_arm_state = fstate.arm_state;

	while(rc_get_state()!=EXITING){
		// re-print header on disarming
		//if(fstate.arm_state==DISARMED && prev_arm_state==ARMED){
		//	__print_header();
		//}

		printf("\r");
		if(settings.printf_arm){
			if(fstate.arm_state == ARMED) printf("%s ARMED %s |",KRED,KNRM);
			else                       printf("%sDISARMED%s|",KGRN,KNRM);
		}
		if(settings.printf_altitude){
			printf("%+5.2f |", fstate.altitude_kf);
		}
		if(settings.printf_rpy){
			printf("%+5.2f|", fstate.roll);
			printf("%+5.2f|", fstate.pitch);
			printf("%+5.2f|", fstate.yaw);
		}
		if(settings.printf_sticks){
			printf("%+5.2f|", user_input.thr_stick);
			printf("%+5.2f|", user_input.roll_stick);
			printf("%+5.2f|", user_input.pitch_stick);
			printf("%+5.2f|", user_input.yaw_stick);
		}
		if(settings.printf_setpoint){
			printf("%+5.2f|", setpoint.altitude);
			printf("%+5.2f|", setpoint.roll);
			printf("%+5.2f|", setpoint.pitch);
			printf("%+5.2f|", setpoint.yaw);
		}
		if(settings.printf_u){
			printf("%+5.2f |", fstate.u[0]);
			printf("%+5.2f |", fstate.u[1]);
			printf("%+5.2f |", fstate.u[2]);
			printf("%+5.2f |", fstate.u[3]);
			printf("%+5.2f |", fstate.u[4]);
			printf("%+5.2f |", fstate.u[5]);
		}
		if(settings.printf_motors){
			printf("%+5.2f|", fstate.m[0]);
			printf("%+5.2f|", fstate.m[1]);
			printf("%+5.2f|", fstate.m[2]);
			printf("%+5.2f|", fstate.m[3]);
			//printf("%+5.2f|", fstate.m[4]);
			//printf("%+5.2f|", fstate.m[5]);
		}
		if(settings.printf_mode){
			print_flight_mode(user_input.flight_mode);
		}

		fflush(stdout);
		prev_arm_state = fstate.arm_state;
		rc_usleep(1000000/PRINTF_MANAGER_HZ);
	}
	return NULL;
}



int printf_init()
{
	if(rc_pthread_create(&printf_manager_thread, __printf_manager_func, NULL,
				SCHED_FIFO, PRINTF_MANAGER_PRI)==-1){
		fprintf(stderr,"ERROR in start_printf_manager, failed to start thread\n");
		return -1;
	}
	rc_usleep(50000);
	return 0;
}


int join_printf_manager_thread(){
	int ret = 0;
	if(initialized){
		// wait for the thread to exit
		ret = rc_pthread_timed_join(printf_manager_thread,NULL,PRINTF_MANAGER_TOUT);
		if(ret==1) fprintf(stderr,"WARNING: printf_manager_thread exit timeout\n");
		else if(ret==-1) fprintf(stderr,"ERROR: failed to join printf_manager thread\n");
	}
	initialized = 0;
	return ret;
}
