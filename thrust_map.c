/*******************************************************************************
* thrust_map.c
*
* Most ESC/motor/propeller combinations provide a highly non-linear map from 
* input to thrust. For the thrust table defined in thrust_map.h, this provides
* the function to translate a desired normalized thrust (0-1) to the necessary
* input (also 0-1).
*******************************************************************************/

#include "thrust_map.h"

float* signal, thrust;
int points;

/*******************************************************************************
* int initialize_thrust_interpolation()
*
* check the thrust map for validity and populate data arrays
*******************************************************************************/
int initialize_thrust_interpolation(){
	int i;
	float max;
	float map[][] = THRUST_MAP;
	// see how many datapoints (rows) are in the user-provided table
	points = sizeof(map)/sizeof(map[0]); 
	
	// sanity checks
	if(points<2){
		printf("ERROR: need at least 2 datapoints in THRUST_MAP\n");
		return -1;
	}
	if(map[0][0] != 0.0){
		printf("ERROR: first row input must be 0.0\n");
		return -1;
	}
	if(map[points-1][0] != 1.0){
		printf("ERROR: last row input must be 1.0\n");
		return -1;
	}
	if(map[0][0] != 0.0){
		printf("ERROR: first row thrust must be 0.0\n");
		return -1;
	}
	if(map[points-1][0] > 0.0){
		printf("ERROR: last row thrust must be > 0.0\n");
		return -1;
	}
	// check that all data is monotonically increasing
	for(i=1;i<points;i++){
		if(map[i][0]<=map[i-1][0] || map[i][1]<=map[i-1][1]){
			printf("ERROR: THRUST_MAP must be monotonically increasing\n");
			return -1;
		}
	}
	
	// create new global array of normalized thrust and inputs
	max = map[points-1][1];
	signal = (float*) malloc(points * sizeof(float));
	thrust = (float*) malloc(points * sizeof(float));
	for(i=0; i<points; i++){
		signal[i] = map[i][0];
		thrust[i] = map[i][1]/max;
	}

	return 0;
}

/*******************************************************************************
* float interpolate_motor_signal(float t)
*
* return the required normalized esc signal for desired normalized thrust t
*******************************************************************************/
float interpolate_motor_signal(float t){
	int i;
	float pos;

	// sanity check
	if(desired>1.0 || desired<0.0){
		printf("ERROR: desired thrust t must be between 0.0 & 1.0\n");
		return -1;
	}
	
	// return quickly for boundary conditions
	if(t==0.0) return 0.0;
	if(t==1.0) return 1.0;

	// scan through the data to pick the upper and lower points to interpolate
	for(i=1; i<points; i++){
		if(t <= thrust[i]){
			pos = (t-thrust[i-1])/(thrust[i]-thrust[i-1]);
			return signal[i-1]+(pos*(signal[i]-signal[i-1]));
		}
	}
	
	printf("ERROR: something in interpolate_motor_signal went wrong\n");
	return -1;
}