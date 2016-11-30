/*******************************************************************************
* fly.c
* 
* James Strawson 2016
* see README.txt for description and use				
*******************************************************************************/

#include <roboticscape-usefulincludes.h>
#include <roboticscape.h>
#include "fly_function_declarations.h"
#include "fly_types.h"
#include "fly_defs.h"
#include "basic_settings.h"


/*******************************************************************************
* 	Global Variables				
*******************************************************************************/
setpoint_t		setpoint;
cstate_t		cstate;
user_input_t	user_input;
imu_data_t		imu_data;
fly_settings_t	settings;

/*******************************************************************************
*	main()
*
*	Initialize the IMU, start all the threads, and wait still something 
*	triggers a shut down.
*******************************************************************************/
int main(int argc, char* argv[]){
	int ret=0; // value to be returned when main() exits

	// load settings first, will be re-read later every re-arm
	if(load_settings_from_file(fly_settings_t* settings)){
		ret = -1;
		goto END;
	}

	// initialize cape hardware
	if(initialize_roboticscape()<0){
		ret = -1;
		goto END;
	}
	rc_set_led(RED,1);
	rc_set_led(GREEN,0);
	rc_set_state(UNINITIALIZED);

	// set up button handler so user can exit by holding pause
	set_pause_pressed_func(&pause_pressed_func);

	// start barometer with no oversampling
	// do this before imu because IMU will use the bus continuously once started
	if(initialize_barometer(BMP_OVERSAMPLE_16,BMP_FILTER_OFF)){
		printf("ERROR: failed to initialize_barometer");
		blink_led(RED,5,3);
		cleanup_cape();
		return -1;
	}

	// start the IMU
	imu_config_t conf = get_default_imu_config();
	conf.dmp_sample_rate = settings.feedback_hz;
	conf.enable_magnetometer = 1;
	conf.orientation = settings.bbb_orientation;

	// now set up the imu for dmp interrupt operation
	if(initialize_imu_dmp(&imu_data, conf)){
		printf("ERROR: failed to initialize IMU\n");
		blink_led(RED,5,3);
		cleanup_cape();
		return -1;
	}


	//initialize other things that are not threads
	if(initialize_thrust_interpolation()<0){
		printf("ERROR: failed to initialize thrust interpolation\n");
		return -1;
	}

	if(start_battery_manager_thread(&cstate, &settings)<0){
		printf("ERROR: failed to start battery manager thread\n");
		return -1;
	}

	if(start_input_manager(&user_input, &settings)<0){
		printf("ERROR: can't start DSM2 radio service\n");
		
		return -1;
	}
	if(start_setpoint_manager(&setpoint, &user_input)<0){
		printf("ERROR: can't start DSM2 radio service\n");
		blink_led(RED,5,3);
		return -1;
	}
	// start printf_thread only if running from a terminal
	if(isatty(fileno(stdout))){
		start_printf_manager(&cstate, &setpoint);
	}


	// set up feedback controller
	initialize_controller(&cstate, &setpoint, &imu_data, &user_input);

	printf("\nTurn your transmitter kill switch UP\n");
	printf("Then move throttle UP then DOWN to arm\n");


	set_state(RUNNING);

	//chill until something exits the program
	while(get_state()!=EXITING){
		usleep(100000);
	}
	

CLEANUP:
	// cleanup before closing
	join_setpoint_manager_thread();
	join_input_manager_thread();
	join_printf_manager_thread();
	join_battery_manager_thread();
	power_off_imu();
	power_off_barometer();

END:
	if(ret!=0){
		blink_led(RED,5,3);
	}
	cleanup_cape();
	return ret;
}







