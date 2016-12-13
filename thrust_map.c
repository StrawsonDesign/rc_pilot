/*******************************************************************************
* thrust_map.c
*
* Most ESC/motor/propeller combinations provide a highly non-linear map from 
* input to thrust. For the thrust table defined in thrust_map.h, this provides
* the function to translate a desired normalized thrust (0-1) to the necessary
* input (also 0-1).
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "thrust_map_defs.h"
#include "fly_types.h"



float* signal;
float* thrust;
int points;

/*******************************************************************************
* int initialize_thrust_(thrust_map_t map)
*
* check the thrust map for validity and populate data arrays
*******************************************************************************/
int initialize_thrust_map(thrust_map_t map){
	int i;
	float max;
	float (*data)[2]; // pointer to constant data

	switch(map){
	case MN1806_1400KV_4S:
		points = mn1806_1400kv_4s_points;
		data = mn1806_1400kv_4s_map;
		break;
	case F20_2300KV_2S:
		points = f20_2300kv_2s_points;
		data = f20_2300kv_2s_map;
		break;
	default:
		printf("ERROR: unknown thrust map\n");
		return -1;
	}

	// sanity checks
	if(points<2){
		printf("ERROR: need at least 2 datapoints in THRUST_MAP\n");
		return -1;
	}
	if(data[0][0] != 0.0){
		printf("ERROR: first row input must be 0.0\n");
		return -1;
	}
	if(data[points-1][0] != 1.0){
		printf("ERROR: last row input must be 1.0\n");
		return -1;
	}
	if(data[0][1] != 0.0){
		printf("ERROR: first row thrust must be 0.0\n");
		return -1;
	}
	if(data[points-1][1] < 0.0){
		printf("ERROR: last row thrust must be > 0.0\n");
		return -1;
	}
	for(i=1;i<points;i++){
		if(data[i][0]<=data[i-1][0] || data[i][1]<=data[i-1][1]){
			printf("ERROR: thrust_map must be monotonically increasing\n");
			return -1;
		}
	}
	// create new global array of normalized thrust and inputs
	free(signal);
	free(thrust);
	signal = (float*) malloc(points * sizeof(float));
	thrust = (float*) malloc(points * sizeof(float));
	max = data[points-1][1];
	for(i=0; i<points; i++){
		signal[i] = data[i][0];
		thrust[i] = data[i][1]/max;
	}
	return 0;
}

/*******************************************************************************
* float map_motor_signal(float* m)
*
* return the required normalized esc signal for desired normalized thrust t
*******************************************************************************/
float map_motor_signal(float m){
	int i;
	float pos;

	// sanity check
	if(m>1.0 || m<0.0){
		printf("ERROR: desired thrust t must be between 0.0 & 1.0\n");
		return -1;
	}

	// return quickly for boundary conditions
	if(m==0.0 || m==1.0) return m; 

	// scan through the data to pick the upper and lower points to interpolate
	for(i=1; i<points; i++){
		if(m <= thrust[i]){
			pos = (m-thrust[i-1])/(thrust[i]-thrust[i-1]);
			return signal[i-1]+(pos*(signal[i]-signal[i-1]));
		}
	}

	printf("ERROR: something in map_motor_signal went wrong\n");
	return -1;
}