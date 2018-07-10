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
#include <rc/button.h>
#include <rc/led.h>
#include <rc/time.h>

#include <settings.h> // contains extern settings variable
#include <thrust_map.h>
#include <mix.h>
#include <input_manager.h>
#include <setpoint_manager.h>
#include <log_manager.h>
#include <printf_manager.h>



#define FAIL(str) \
fprintf(stderr, str); \
rc_led_set(RC_LED_GREEN,0); \
rc_led_blink(RC_LED_RED,4.0,2.0); \
return -1;



/**
* If the user holds the pause button for 2 seconds, set state to exiting which
* triggers the rest of the program to exit cleanly.
*/
void on_pause_press()
{
	int i=0;
	const int quit_check_us = 100000;
	const int samples = 2000000/quit_check_us;

	// toggle betewen paused and running modes
	if(rc_get_state()==RUNNING){
		rc_set_state(PAUSED);
		printf("PAUSED\n");
	}
	else if(rc_get_state()==PAUSED){
		rc_set_state(RUNNING);
		printf("RUNNING\n");
	}
	fflush(stdout);

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(quit_check_us);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED){
			return;
		}
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}




/**
 * Initialize the IMU, start all the threads, and wait still something triggers
 * a shut down.
 *
 * @return     0 on success, -1 on failure
 */
int main()
{
	// make sure another instance isn't running
	// return value -3 means a root process is running and we need more
	// privileges to stop it.
	if(rc_kill_existing_process(2.0)==-3) return -1;

	// start with both LEDs off
	if(rc_led_set(RC_LED_GREEN, 0)==-1){
		fprintf(stderr, "ERROR in main(), failed to set RC_LED_GREEN\n");
		return -1;
	}
	if(rc_led_set(RC_LED_RED, 0)==-1){
		fprintf(stderr, "ERROR in main() failed to set RC_LED_RED\n");
		return -1;
	}


	// first things first, load settings which may be used during startup
	if(settings_load_from_file()){
		FAIL("ERROR: failed to load settings file quitting fly\n")
	}
	printf("Loaded settings\n");

	// do initialization not involving threads
	printf("initializing thrust map\n");
	if(thrust_map_init(settings.thrust_map)<0){
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

	// initialize cape hardware, this prints an error itself if unsuccessful
	printf("initializing servos\n");
	if(rc_servo_init()==-1) return -1;
	printf("initializing adc\n");
	if(rc_adc_init()==-1) return -1;

	// start signal handler so threads can exit cleanly
	printf("initializing signal handler\n");
	if(rc_enable_signal_handler()<0){
		fprintf(stderr,"ERROR: failed to complete rc_enable_signal_handler\n");
		return -1;
	}

	// start threads
	printf("initializing DSM and input_manager\n");
	if(input_manager_init()<0){
		printf("ERROR: failed to initialize input_manager\n");
		return -1;
	}

	// initialize buttons
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to init buttons\n");
		return -1;
	}

	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,NULL);

	// set up feedback controller
	printf("initializing feedback controller\n");
	feedback_init();


	// initalize log_manager
	if(log_manager_init()<0){
		printf("ERROR: failed to initialize input_manager\n");
		return -1;
	}

	if(isatty(fileno(stdout))){
	 	printf("initializing printf manager\n");
	}

	printf("\nTurn your transmitter kill switch to arm.\n");
	printf("Then move throttle UP then DOWN to arm controller\n\n");

	 // start printf_thread if running from a terminal
	 // if it was started as a background process then don't bother
	 if(isatty(fileno(stdout))){
	 	printf_init();
	 }

	// final setup
	rc_make_pid_file();
	rc_set_state(RUNNING);

	// set state to running and chill until something exits the program
	rc_set_state(RUNNING);
	while(rc_get_state()!=EXITING){
		usleep(500000);
	}

	printf("cleaning up\n");
	feedback_cleanup();
	setpoint_manager_cleanup();
	input_manager_cleanup();
	log_manager_cleanup();
	return 0;
}







