/*******************************************************************************
* mixing_matrix.c
*
* James Strawson 2016
* MultiRotors are controlled by mixing roll, pitch, yaw, and throttle
* control oututs, a linear combination of which forms the control output
* to each motor. The coefficients to this combination is stored in a 
* mixing matrix based on rotor layout.
* Also included here are functions to parse configuration strings 
* and do the actual mixing.
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include "fly_types.h"
#include "mixing_matrix_defs.h"

float (*mix_matrix)[6];
int initialized;
int rotors;
int dof;

/*******************************************************************************
* int initialize_mixing_matrix(layout_t layout)
* 
* For a given number of rotors, layout character ('X','+') and degrees of
* freedom in control input (4 or 6), this selects the correct predefined
* mixing matrix from mixing_matrix_defs.h. The matrix is kept locally in
* mixing_matrix.c to prevent accidental misuse or modification. Use the other
* functions here to interface with it.
*******************************************************************************/
int initialize_mixing_matrix(layout_t layout){

	switch(layout){
	case LAYOUT_6DOF_ROTORBITS:
		rotors = 6;
		dof = 6;
		mix_matrix = mix_rotorbits_6dof;
		break;
	case LAYOUT_4X:
		rotors = 4;
		dof = 4;
		mix_matrix = mix_4x;
		break;
	case LAYOUT_4PLUS:
		rotors = 4;
		dof = 4;
		mix_matrix = mix_4plus;
		break;
	case LAYOUT_6X:
		rotors = 6;
		dof = 4;
		mix_matrix = mix_6x;
		break;
	case LAYOUT_8X:
		rotors = 8;
		dof = 4;
		mix_matrix = mix_8x;
		break;
	default:
		printf("ERROR: unknown rotor layout\n");
		return -1;
	}

	initialized = 1;
	return 0;
}

/************************************************************************
* int mix_all_controls(float u[6], float* mot)
*	
* fills the vector mot with the linear combination of roll pitch
* yaw and throttle based on mixing matrix. if dof=6, X and Y are also
* added. Outputs are blindly saturated between 0 and 1. This is for rudimentary 
* mixing and testing only. It is recommended to check for saturation for each 
* input with check_channel_saturation then add inputs sequentially with 
* add_mixed_input() instead.
*******************************************************************************/
int mix_all_controls(float u[6], float* mot){
	int i,j;
	if(initialized!=1){
		printf("ERROR: mix_all_controls, mixing matrix not set yet\n");
		return -1;
	}
	// sum control inputs
	for(i=0;i<rotors;i++){
		mot[i]=0.0;
		for(j=0;j<6;j++){
			mot[i]+=mix_matrix[i][j]*u[j];
		}
	}
	for(i=0;i<rotors;i++){
		if(mot[i]>1.0) mot[i]=1.0;
		else if(mot[i]<0.0) mot[i]=0.0;
	}
	return 0;
}

/*******************************************************************************
* int check_channel_saturation(int ch, float* mot, float* min, float* max)
*	
* calculates the maximum (always positive) and minimum (always negative) inputs
* that could be applied to a single channel given the current state of the
* motors. Current motor inputs must be in [0,1] inclusive or an error will be
* thrown. That condition is guaranteed by add_mixed_input(). 
*******************************************************************************/
int check_channel_saturation(int ch, float* mot, float* min, float* max){
	int i;
	float tmp;

	if(initialized!=1){
		printf("ERROR: in check_channel_saturation, mix matrix not set yet\n");
		return -1;
	}
	if(ch<0 || ch>=MAX_INPUTS){
		printf("ERROR: in check_channel_saturation, ch out of bounds\n");
		return -1;
	}

	// make sure motors are not already saturated
	for(i=0;i<rotors;i++){
		if(mot[i]>1.0 || mot[i]<0.0){
			printf("ERROR: motor channel already out of bounds\n");
			return -1;
		}
	}

	// find max positive input
	float new_max = FLT_MAX;
	for(i=0;i<rotors;i++){
		// if mix channel is 0, impossible to saturate
		if(mix_matrix[i][ch]==0.0) continue;
		// for positive entry in mix matrix
		if(mix_matrix[i][ch]>0.0)	tmp = (1.0-mot[i])/mix_matrix[i][ch];
		// for negative entry in mix matrix
		else tmp = -mot[i]/mix_matrix[i][ch];
		// set new upper limit if lower than current
		if(tmp<new_max) new_max = tmp;
	}

	// find min (most negative) input
	float new_min = -FLT_MAX; 
	for(i=0;i<rotors;i++){
		// if mix channel is 0, impossible to saturate
		if(mix_matrix[i][ch]==0.0) continue;
		// for positive entry in mix matrix
		if(mix_matrix[i][ch]>0.0)	tmp = -mot[i]/mix_matrix[i][ch];
		// for negative entry in mix matrix
		else tmp = (1.0-mot[i])/mix_matrix[i][ch];
		// set new upper limit if lower than current
		if(tmp>new_min) new_min = tmp;
	}

	*min = new_min;
	*max = new_max;
	return 0;
}


/*******************************************************************************
* int add_mixed_input(float u, int ch, float* mot)
*
* Mixes the control input u for a single channel ch corresponding to throttle, 
* roll pitch etc to the existing motor array mot. The motor output is saturated
* between 0 and 1 for safety only. Input u should be checked for saturation 
* validity with check_channel_saturation() first to avoid 
*******************************************************************************/
int add_mixed_input(float u, int ch, float* mot){
	int i;
	if(initialized!=1){
		printf("ERROR: in add_mixed_input, mix matrix not set yet\n");
		return -1;
	}
	if(ch<0 || ch>=MAX_INPUTS){
		printf("ERROR: in add_mixed_input, ch out of bounds\n");
		return -1;
	}
	// add inputs
	for(i=0;i<rotors;i++){
		mot[i] += u*mix_matrix[i][ch];
		if(mot[i]>1.0) mot[i]=1.0;
		else if(mot[i]<0.0) mot[i]=0.0;
	}
	return 0;
}


