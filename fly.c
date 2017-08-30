/*******************************************************************************
* fly.c
* 
* James Strawson 2016
* see README.txt for description and use				
*******************************************************************************/

#include <roboticscape.h>
#include <stdio.h>
#include <unistd.h>
#include "fly_function_declarations.h"
#include "fly_types.h"
#include "fly_defs.h"


/*******************************************************************************
* 	Global Variables				
*******************************************************************************/
setpoint_t	setpoint;
cstate_t	cstate;
user_input_t	user_input;
rc_imu_data_t	imu_data;
fly_settings_t	settings;

/*******************************************************************************
* main()
*
* Initialize the IMU, start all the threads, and wait still something 
* triggers a shut down.
*******************************************************************************/
int main(){
	// initialize cape hardware, this prints an error itself if unsuccessful
	printf("initializing rc lib\n");
	if(rc_initialize()){
		printf("ERROR: RC lib failed to initialize\n");
		return -1;
	}

	printf("RC lib initialized\n");

/*
	if(load_settings_from_file(&settings)){
		printf("ERROR: invalid settings file, quitting fly\n");
		rc_blink_led(RED,5,3);
		return -1;
	}
	printf("Loaded settings\n");

	// set red led to indicate initialization has started
	rc_set_led(RED,1);
	rc_set_led(GREEN,0);
	// set state to UNINITIALIZED, although initialize_roboticscape does this
	rc_set_state(UNINITIALIZED); 
	
	// set up button handler so user can exit by holding pause
	printf("setting pause pressed in fly\n");
	rc_set_pause_pressed_func(&pause_pressed_func);

	// do initialization not involving threads
	printf("initializing thrust map\n");
	if(initialize_thrust_map(settings.thrust_map)<0){
		printf("ERROR: failed to initialize thrust map\n");
		rc_blink_led(RED,5,3);
		return -1;
	}
	if(initialize_mixing_matrix(settings.layout)<0){
		printf("ERROR: failed to initialize thrust map\n");
		rc_blink_led(RED,5,3);
		return -1;
	}

	// start barometer with max oversampling before IMU since imu will hog
	// i2c bus once setup, barometer will be left alone till we call it.
	if(rc_initialize_barometer(BMP_OVERSAMPLE_16,BMP_FILTER_OFF)){
		printf("ERROR: failed to initialize_barometer");
		rc_blink_led(RED,5,3);
		rc_cleanup();
		return -1;
	}

	// start the IMU
	rc_imu_config_t conf = rc_default_imu_config();
	conf.dmp_sample_rate = settings.feedback_hz;
	conf.enable_magnetometer = 1;
	conf.orientation = settings.bbb_orientation;

	// now set up the imu for dmp interrupt operation
	if(rc_initialize_imu_dmp(&imu_data, conf)){
		printf("initialize_imu_failed\n");
		rc_blink_led(RED,5,3);
		rc_cleanup();
		return -1;
	}
	// start threads
	if(start_input_manager(&user_input, &settings)<0){
		printf("ERROR: can't start input_manager\n");
		rc_blink_led(RED,5,3);
		return -1;
	}
	if(start_setpoint_manager(&setpoint, &user_input, &cstate, &settings)<0){
		printf("ERROR: can't start setpoint_manager\n");
		rc_blink_led(RED,5,3);
		return -1;
	} 
	// set up feedback controller
	initialize_controller(&cstate, &setpoint, &imu_data, &settings);

	// print header before starting printf thread
	printf("\nTurn your transmitter kill switch UP\n");
	printf("Then move throttle UP then DOWN to arm\n");

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		start_printf_manager(&cstate, &setpoint, &user_input, &settings);
	}

	// set state to running and chill until something exits the program
	rc_set_state(RUNNING);
	while(rc_get_state()!=EXITING){
		usleep(100000);
	}
	
	// cleanup before closing
	join_setpoint_manager_thread();
	join_input_manager_thread();
	join_printf_manager_thread();
*/
	rc_power_off_imu();
	rc_power_off_barometer();
	rc_cleanup();
	return 0;
}







