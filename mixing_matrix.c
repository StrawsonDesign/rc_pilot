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

#include "basic_settings.h"

float mix_matrix[MAX_INPUTS][MAX_ROTORS];
int initialized;

/*******************************************************************************
* int initialize_mixing_matrix()
* 
* For a given number of rotors, layout character ('X','+') and degrees of
* freedom in control input (4 or 6), this selects the correct predefined
* mixing matrix from mixing_matrix_defs.h. The matrix is kept locally in
* mixing_matrix.c to prevent accidental misuse or modification. Use the other
* functions here to interface with it.
*******************************************************************************/
int initialize_mixing_matrix(){
	int i,j;

	// start with sanity checks
	// current 4, 6, 8 rotors supported with + and X layouts
	switch(ROTORS){
		case 4: break;
		case 6: break;
		case 8: break;
		default:
			printf("ERROR, ROTORS must be 4, 6, or 8\n");
			return -1;
	}
	switch(ROTOR_LAYOUT){
		case 'x': layout='X';
		case 'X': break;
		case '+': break;
		default:
			printf("ERROR, layout must be X or +\n");
			return -1;
	}

	if(DOF==6){
		if(ROTORS!=6 || ROTOR_LAYOUT!='X'){
			printf("ERROR: 6dof control only works with 6X layout\n");
			return -1;
		}
	}
	else if(DOF!=4){
		printf("ERROR: dof must be 4 or 6\n")
		return -1;
	}
	
	// select the right mix values
	if(DOF==6) float new[][] = MIX_6DOF_HEX;
	else if(ROTORS==4 && ROTOR_LAYOUT=='X') float new[][] = MIX_4X;
	else if(ROTORS==4 && ROTOR_LAYOUT=='+') float new[][] = MIX_4PLUS;
	else if(ROTORS==6 && ROTOR_LAYOUT=='X') float new[][] = MIX_6X;
	else if(ROTORS==6 && ROTOR_LAYOUT=='+') float new[][] = MIX_6PLUS;
	else if(ROTORS==8 && ROTOR_LAYOUT=='X') float new[][] = MIX_8X;
	else if(ROTORS==8 && ROTOR_LAYOUT=='+') float new[][] = MIX_8PLUS;
	else{
		printf("ERROR:invalid rotor/layout combination\n");
		return -1;
	}
	// now fill the the mix matrix appropriately
	for(i=0;i<DOF;i++){
		for(j=0;j<ROTORS;j++){
			mix_matrix[i][j] = new[i][j];
		}
	}

	initialized = 1;
	return 0;
}

/************************************************************************
* int mix_all_controls(float* u, float* mot)
*	
* fills the vector mot with the linear combination of roll pitch
* yaw and throttle based on mixing matrix. if dof=6, X and Y are also
* added. No attention is paid to saturation. This is for rudimentary 
* mixing, it is recommended to check for saturation for each input with
* check_channel_saturation then add inputs sequentially with add_mixed_input()
*******************************************************************************/
int mix_all_controls(float* u, float* mot){
	int i,j;
	if(initialized!=1){
		printf("ERROR: mix_all_controls, mixing matrix not set yet\n");
		return -1;
	}
	// sum control inputs
	for(i=0;i<ROTORS;i++){
		mot[i]=0;
		for(j=0;j<DOF;j++){
			mot[i]+=mix_matrix[j][i]*u[j]
		}
	}
	return 0;
}

/*******************************************************************************
* int check_channel_saturation(int ch, float* mot, float* min, float* max)
*	
* checks to see if the desired input u on channel ch corresponding to throttle,
* roll, pitch, etc would saturate a motor in the given motor array mot. If 
* saturation would occur, it returns the maximum allowed input u.
*******************************************************************************/
int check_channel_saturation(int ch, float* mot, float* min, float* max){
	int i;
	float trial[MAX_ROTORS];
	float new_min = -FLT_MAX; 
	float new_max = FLT_MAX;
	float new_u = u; // new u to be returned

	if(initialized!=1){
		printf("ERROR: in check_channel_saturation, mix matrix not set yet\n");
		return -1;
	}
	if(ch<0 || ch>=DOF){
		printf("ERROR: in check_channel_saturation, ch out of bounds\n");
		return -1;
	}
	// check for upper saturation
	for(i=0;i<ROTORS;i++){
		if(mix_matrix[ch][i]==0.0) continue;
		else tmp = (1-mot[i])/mix_matrix[ch][i];
		if(tmp<new_max) new_max = tmp;
	}
	// check for lower saturation
	for(i=0;i<ROTORS;i++){
		if(mix_matrix[ch][i]==0.0) continue;
		else tmp = mot[i]/mix_matrix[ch][i];
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
* roll pitch etc to the existing motor array mot. No saturation is done, the
* input u should be checked for saturation validity with 
* check_channel_saturation() first.
*******************************************************************************/
int add_mixed_input(float u, int ch, float* mot){
	int i;
	if(initialized!=1){
		printf("ERROR: in add_mixed_input, mix matrix not set yet\n");
		return -1;
	}
	if(ch<0 || ch>=ROTORS){
		printf("ERROR: in add_mixed_input, ch out of bounds\n");
		return -1;
	}
	// add inputs
	for(i=0;i<ROTORS;i++) mot[i] += u*mix_matrix[ch][i];
	return 0;
}


