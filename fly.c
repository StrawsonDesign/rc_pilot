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

/*******************************************************************************
*	main()
*
*	Initialize the IMU, start all the threads, and wait still something 
*	triggers a shut down.
*******************************************************************************/
int main(int argc, char* argv[]){
	// initialize cape hardware
	if(initialize_cape()<0){
		blink_led(RED,5,3);
		return -1;
	}
	set_led(RED,1);
	set_led(GREEN,0);
	set_state(UNINITIALIZED);
	
	// set up button handler so user can exit by holding pause
	set_pause_pressed_func(&pause_pressed_func);
	// set up feedback controller before starting sensors
	initialize_controller(&cstate, &setpoint, &imu_data, &user_input);

	if(initialize_thrust_interpolation()<0){
		printf("ERROR: failed to initialize thrust interpolation\n");
		return -1;
	}
	if(start_input_manager(&user_input)<0){
		printf("ERROR: can't start DSM2 radio service\n");
		blink_led(RED,5,3);
		return -1;
	}
	if(start_setpoint_manager(&setpoint, &user_input)<0){
		printf("ERROR: can't start DSM2 radio service\n");
		blink_led(RED,5,3);
		return -1;
	} 

	// start barometer with no oversampling
	if(initialize_barometer(BMP_OVERSAMPLE_16,BMP_FILTER_OFF)){
		printf("ERROR: failed to initialize_barometer");
		blink_led(RED,5,3);
		cleanup_cape();
		return -1;
	}
	// start the IMU
	imu_config_t conf = get_default_imu_config();
	conf.dmp_sample_rate = SAMPLE_RATE_HZ;
	conf.enable_magnetometer = 1;
	conf.orientation = BBB_ORIENTATION;
	
	// now set up the imu for dmp interrupt operation
	if(initialize_imu_dmp(&imu_data, conf)){
		printf("initialize_imu_failed\n");
		blink_led(RED,5,3);
		cleanup_cape();
		return -1;
	}
	
	
	printf("\nTurn your transmitter kill switch UP\n");
	printf("Then move throttle UP then DOWN to arm\n");

	
	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		start_printf_manager(&cstate, &setpoint);
	}

	set_state(RUNNING);

	//chill until something exits the program
	while(get_state()!=EXITING){
		usleep(100000);
	}
	
	// cleanup before closing
	join_setpoint_manager_thread();
	join_input_manager_thread();
	join_printf_manager_thread();
	power_off_imu();
	power_off_barometer();
	cleanup_cape();	// de-initialize cape hardware
	return 0;
}







