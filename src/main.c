/**
* @file fly.c
*
* see README.txt for description and use
**/

#include <stdio.h>
#include <unistd.h>

#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/bmp.h>

#include <settings.h> // contains extern settings variable
#include <thrust_map.h>
#include <mix.h>
#include <input_manager.h>
#include <setpoint_manager.h>


/**
 * Initialize the IMU, start all the threads, and wait still something triggers
 * a shut down.
 *
 * @return     0 on success, -1 on failure
 */
int main()
{
	// first things first, load settings which may be used during startup
	if(settings_load_from_file()){
		printf("ERROR: failed to load settings file quitting fly\n");
		return -1;
	}
	printf("Loaded settings\n");

	// do initialization not involving threads
	printf("initializing thrust map\n");
	if(thrust_map_init()<0){
		fprintf(stderr,"ERROR: failed to initialize thrust map\n");
		return -1;
	}
	printf("initializing mixing matrix\n");
	if(mix_init(settings.layout)<0){
		fprintf(stderr,"ERROR: failed to initialize mixing matrix\n");
		return -1;
	}
	printf("initializing setpoint_manager\n");
	if(setpoint_manager_init()<0){
		fprintf(stderr, "ERROR: failed to initialize setpoint_manager\n");
		return -1;
	}

	// start threads
	if(input_manager_init()<0){
		printf("ERROR: failed to initialize input_manager\n");
		return -1;
	}
	// if(start_setpoint_manager(&setpoint, &user_input, &cstate, &settings)<0){
	// 	printf("ERROR: can't start setpoint_manager\n");
	// 	rc_blink_led(RED,5,3);
	// 	return -1;
	// }

/*
	// initialize cape hardware, this prints an error itself if unsuccessful
	if(rc_servo_init()==-1) return -1;
	if(rc_adc_init()==-1) return -1;

	// make sure another instance isn't running
	// return value -3 means a root process is running and we need more
	// privileges to stop it.
	if(rc_kill_existing_process(2.0)==-3) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()<0){
		fprintf(stderr,"ERROR: failed to complete rc_enable_signal_handler\n");
		return -1;
	}

	// initialize buttons
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to init buttons\n");
		return -1;
	}

	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,on_pause_release);
	rc_button_set_callbacks(RC_BTN_PIN_MODE,NULL,on_mode_release);

	// start with both LEDs off
	if(rc_led_set(RC_LED_GREEN, 0)==-1){
		fprintf(stderr, "ERROR in rc_blink, failed to set RC_LED_GREEN\n");
		return -1;
	}
	if(rc_led_set(RC_LED_RED, 0)==-1){
		fprintf(stderr, "ERROR in rc_blink, failed to set RC_LED_RED\n");
		return -1;
	}

	// final setup
	rc_make_pid_file();
	rc_set_state(RUNNING);
	mode = 0;
	printf("\nPress mode to change blink rate\n");
	printf("hold pause button to exit\n");


	printf("RC lib initialized\n");



	// set red led to indicate initialization has started
	rc_set_led(RED,1);
	rc_set_led(GREEN,0);
	// set state to UNINITIALIZED, although initialize_roboticscape does this
	rc_set_state(UNINITIALIZED);

	// set up button handler so user can exit by holding pause
	rc_set_pause_pressed_func(pause_pressed_func);




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
		usleep(500000);
	}

	printf("cleaning up\n");
	// cleanup before closing
	join_setpoint_manager_thread();
	join_input_manager_thread();
	join_printf_manager_thread();

	rc_mpu_power_off();
	rc_bmp_power_off();

	*/
	return 0;
}







