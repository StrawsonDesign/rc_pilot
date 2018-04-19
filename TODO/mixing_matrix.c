/**
 * @file mixing_matrix.c
 *
 * MultiRotors are controlled by mixing roll, pitch, yaw, and throttle control
 * oututs, a linear combination of which forms the control output to each motor.
 * The coefficients to this combination is stored in a mixing matrix based on
 * rotor layout. Also included here are functions to parse configuration strings
 * and do the actual mixing.
 */

#include <stdio.h>
#include <stdlib.h>
#include <float.h> // for FLT_MAX
#include <mixing_matrix.h>
#include <fly_types.h>

float (*mix_matrix)[6];
int initialized;
int rotors;
int dof;


int initialize_mixing_matrix(rotor_layout_t layout)
{
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
		fprintf(stderr,"ERROR: unknown rotor layout\n");
		return -1;
	}

	initialized = 1;
	return 0;
}


int mix_all_controls(float u[6], float* mot)
{
	int i,j;
	if(initialized!=1){
		fprintf(stderr,"ERROR: mix_all_controls, mixing matrix not set yet\n");
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


int check_channel_saturation(int ch, float* mot, float* min, float* max)
{
	int i;
	float tmp;

	if(initialized!=1){
		fprintf(stderr,"ERROR: in check_channel_saturation, mix matrix not set yet\n");
		return -1;
	}
	if(ch<0 || ch>=MAX_INPUTS){
		fprintf(stderr,"ERROR: in check_channel_saturation, ch out of bounds\n");
		return -1;
	}

	// make sure motors are not already saturated
	for(i=0;i<rotors;i++){
		if(mot[i]>1.0 || mot[i]<0.0){
			fprintf(stderr,"ERROR: motor channel already out of bounds\n");
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


int add_mixed_input(float u, int ch, float* mot)
{
	int i;
	if(initialized!=1){
		fprintf(stderr,"ERROR: in add_mixed_input, mix matrix not set yet\n");
		return -1;
	}
	if(ch<0 || ch>=MAX_INPUTS){
		fprintf(stderr,"ERROR: in add_mixed_input, ch out of bounds\n");
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


