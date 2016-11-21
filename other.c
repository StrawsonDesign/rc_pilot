/*******************************************************************************
* other.c
*
* This is a homeless shelter for poor functions without a place to live.
*******************************************************************************/

#include <roboticscape.h>
#include <roboticscape-usefulincludes.h>
#include "basic_settings.h"
#include "fly_function_declarations.h"

/*******************************************************************************
* float apply_deadzone(float in, float zone)
*
* Applies a dead zone to an input stick in. in is supposed to range from -1 to 1
* the dead zone is centered around 0. zone specifies the distance from 0 the
* zone extends.
*******************************************************************************/
float apply_deadzone(float in, float zone){
	if(zone<0){
		printf("ERROR: dead zone must be >0\n");
		return in;
	}
	// inside dead zone, return 
	if(fabs(in)<zone) return 0.0;
	if(in>0) return ((in-zone)/(1.0-zone)) + zone;
	else	 return ((in+zone)/(1.0-zone)) - zone;
}

/*******************************************************************************
* int set_motors_to_zero()
*
* sends signal 0 to all motor channels
*******************************************************************************/
int set_motors_to_zero(){
	int i;
	for(i=0;i<ROTORS;i++) send_esc_pulse_normalized(i+1,0);
	return 0;
}


/*******************************************************************************
* int on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
int on_pause_released(){
	// toggle betewen paused and running modes
	if(get_state()==PAUSED)	set_state(RUNNING);
	return 0;
}

/*******************************************************************************
* int pause_pressed_func()
*
* Disarm controller on momentary press.
* If the user holds the pause button for BUTTON_EXIT_TIME_S, exit cleanly.
*******************************************************************************/
int pause_pressed_func(){
	int i;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds
	
	if(get_state()==EXITING) return 0;
	
	// always disarm controller as soon as pause button is pressed
	disarm_controller();
	if(get_state()==RUNNING)	set_state(PAUSED);

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		usleep(us_wait/samples);
		if(get_pause_button() == RELEASED) return 0;
	}
	printf("long press detected, shutting down\n");
	blink_led(RED, 5,1);
	set_state(EXITING);
	return 0;
}

