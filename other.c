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
