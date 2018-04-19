/**
 * @file printf_manager.c
 */

#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include <rc/state.h>
#include <rc/time.h>
#include <rc/pthread_helpers.h>

#include <fly_defs.h>
#include <printf_manager.h>

cstate_t*	cs;	// pointer to external core_state_t struct
setpoint_t*	sp;	// pointer to external setpoint struct
user_input_t*	ui;	// pointer to external user input struct
fly_settings_t*	set;	// pointer to external settings struct

pthread_t pthread;


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


void* printf_manager_func(__attribute__ ((unused)) void* ptr){
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
		rc_usleep(1000000/PRINTF_MANAGER_HZ);
	}
	return NULL;
}



int start_printf_manager(cstate_t* cstate, setpoint_t* setpoint,
			user_input_t* user_input, fly_settings_t* settings){
	cs = cstate;
	sp = setpoint;
	ui = user_input;
	set = settings;
	if(rc_pthread_create(&pthread, printf_manager_func, NULL, SCHED_FIFO, printf_MANAGER_PRI)<0){
		fprintf(stderr,"ERROR in start_printf_manager, failed to start thread\n");
		return -1;
	}
	rc_usleep(1000);
	return 0;
}


int join_printf_manager_thread(){
	// wait for the thread to exit
	int ret = rc_pthread_timed_join(pthread,NULL,PRINTF_MANAGER_TOUT);
	if(ret==1) fprintf(stderr,"WARNING: printf_manager_thread exit timeout\n");
	else if(ret==-1) fprintf(stderr,"ERROR: failed to joing printf_manager thread\n");
	return ret;
}
