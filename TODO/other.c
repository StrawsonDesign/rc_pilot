/**
* other.c
*
* This is a homeless shelter for poor functions without a place to live.
**/

#include <roboticscape.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include "fly_function_declarations.h"

/**
* float apply_deadzone(float in, float zone)
*
* Applies a dead zone to an input stick in. in is supposed to range from -1 to 1
* the dead zone is centered around 0. zone specifies the distance from 0 the
* zone extends.
**/
float apply_deadzone(float in, float zone)
{
	if(zone<=0.0){
		printf("ERROR: dead zone must be > 0.0\n");
		return in;
	}
	// inside dead zone, return
	if(fabs(in)<zone) return 0.0;
	if(in>0.0)	return ((in-zone)/(1.0-zone)) + zone;
	else		return ((in+zone)/(1.0-zone)) - zone;
}

/**
 * Make the Pause button toggle between paused and running states.
 */
void on_pause_release()
{
	// toggle betewen paused and running modes
	if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/**
* If the user holds the pause button for 2 seconds, set state to exiting which
* triggers the rest of the program to exit cleanly.
*/
void on_pause_press()
{
	int i;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	// always disarm controller as soon as pause button is pressed
	disarm_controller();

	if(rc_get_state()==EXITING){
		return;
	}
	if(rc_get_state()==RUNNING){
		rc_set_state(PAUSED);
	}

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	rc_led_blink(RC_LED_RED, 5,1);
	return;
}

