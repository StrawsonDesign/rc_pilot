/*******************************************************************************
* fly.c
* 
* James Strawson 2016
* see README.txt for description and use				
*******************************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <roboticscape.h>
#include "fly_function_declarations.h"
#include "fly_types.h"
#include "fly_defs.h"


/*******************************************************************************
* 	Global Variables				
*******************************************************************************/
setpoint_t		setpoint;
cstate_t		cstate;
user_input_t	user_input;
imu_data_t		imu_data;
fly_settings_t	settings;

/*******************************************************************************
* main()
*
* Initialize the IMU, start all the threads, and wait still something 
* triggers a shut down.
*******************************************************************************/
int main(){

	if(load_settings_from_file(&settings)){
		rc_blink_led(RED,5,3);
		printf("ERROR: invalid settings file, quitting fly\n");
		return -1;
	}

	// initialize cape hardware, this prints an error itself if unsuccessful
	if(initialize_roboticscape()<0){
		rc_blink_led(RED,5,3);
		return -1;
	}
	// set red led to indicate initialization has started
	rc_set_led(RED,1);
	rc_set_led(GREEN,0);
	// set state to UNINITIALIZED, although initialize_roboticscape does this
	rc_set_state(UNINITIALIZED); 
	
	// set up button handler so user can exit by holding pause
	set_pause_pressed_func(&pause_pressed_func);

	// do initialization not involving threads
	if(initialize_thrust_map(settings.thrust_map)<0){
		printf("ERROR: failed to initialize thrust map\n");
		rc_blink_led(RED,5,3);
		return -1;
	}
	if(initialize_mix_matrix(settings.layout)<0){
		printf("ERROR: failed to initialize thrust map\n");
		rc_blink_led(RED,5,3);
		return -1;
	}

	// start barometer with max oversampling before IMU since imu will hog
	// i2c bus once setup, barometer will be left alone till we call it.
	if(initialize_barometer(BMP_OVERSAMPLE_16,BMP_FILTER_OFF)){
		printf("ERROR: failed to initialize_barometer");
		rc_blink_led(RED,5,3);
		cleanup_roboticscape();
		return -1;
	}

	// start the IMU
	imu_config_t conf = get_default_imu_config();
	conf.dmp_sample_rate = settings.feedback_hz;
	conf.enable_magnetometer = 1;
	conf.orientation = settings.bbb_orientation;

	// now set up the imu for dmp interrupt operation
	if(initialize_imu_dmp(&imu_data, conf)){
		printf("initialize_imu_failed\n");
		rc_blink_led(RED,5,3);
		cleanup_roboticscape();
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
	initialize_controller(&cstate, &setpoint, &imu_data, &user_input);

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
	power_off_imu();
	power_off_barometer();
	cleanup_roboticscape();
	return 0;
}







