/*******************************************************************************
* other.c
*
* This is a homeless shelter for poor functions without a place to live.
*******************************************************************************/


/*******************************************************************************
* float apply_deadzone(float in, float zone)
*
* Applies a dead zone to an input stick in. in is supposed to range from -1 to 1
* the dead zone is centered around 0. zone specifies the distance from 0 the
* zone extends.
*******************************************************************************/
float apply_deadzone(float in, float zone){
	float tmp;
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
* int pause_pressed_func()
*
* Disarm controller on momentary press.
* If the user holds the pause button for BUTTON_EXIT_TIME_S, exit cleanly.
*******************************************************************************/
int pause_pressed_func(){
	int i;
	const int samples = BUTTON_EXIT_CHECK_HZ * BUTTON_EXIT_TIME_S;
	
	if(get_state()==EXITING) return 0;
	
	// always disarm controller as soon as pause button is pressed
	disarm_controller();
	// now wait to see if the user wants to shut down the program
	i=0;
	while(i<samples){
		if(get_pause_button_state() == RELEASED) return 0;
		i++;
		usleep(1000000/BUTTON_EXIT_CHECK_HZ);
	}
	printf("long press detected, shutting down\n");
	blink_led(RED, 5,1);
	set_state(EXITING);
	return 0;
}
