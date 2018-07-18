/**
 * @file mixing_matrix.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <float.h> // for DBL_MAX
#include <mix.h>


/**
 * Most popular: 4-rotor X layout like DJI Phantom and 3DR Iris
 * top view:
 *  4   1       cw ccw      X   Z down
 *    X                     ^
 *  3   2       ccw cw      + > Y
 *
 * columns: X Y Z Roll Pitch Yaw
 * rows: motors 1-4
 */
static double mix_4x[][6] = { \
{0.0,   0.0,  -1.0,  -0.5,   0.5,   0.5},\
{0.0,   0.0,  -1.0,  -0.5,  -0.5,  -0.5},\
{0.0,   0.0,  -1.0,   0.5,  -0.5,   0.5},\
{0.0,   0.0,  -1.0,   0.5,   0.5,  -0.5}};

/**
 * less popular: 4-rotor + layout
 *
 * top view:
 *    1       ccw      	X   Z down
 *  4 + 2   cw   cw     ^
 *    3       ccw      	+ > Y
 * columns: X Y Z Roll Pitch Yaw
 * rows: motors 1-4
 */
static double mix_4plus[][6] = { \
{0.0,   0.0,  -1.0,   0.0,   0.5,   0.5},\
{0.0,   0.0,  -1.0,  -0.5,   0.0,  -0.5},\
{0.0,   0.0,  -1.0,   0.0,  -0.5,   0.5},\
{0.0,   0.0,  -1.0,   0.5,   0.0,  -0.5}};

/*
 * 6X like DJI S800
 *
 * top view:
 *  6  1       cw ccw      X   Z down
 * 5    2    ccw     cw    ^
 *  4  3       cw ccw      + > Y
 * columns: X Y Z Roll Pitch Yaw
 * rows: motors 1-6
 */
static double mix_6x[][6] = { \
{0.0,   0.0,  -1.0,  -0.25,   0.5,   0.5},\
{0.0,   0.0,  -1.0,  -0.50,   0.0,  -0.5},\
{0.0,   0.0,  -1.0,  -0.25,  -0.5,   0.5},\
{0.0,   0.0,  -1.0,   0.25,  -0.5,  -0.5},\
{0.0,   0.0,  -1.0,   0.50,   0.0,   0.5},\
{0.0,   0.0,  -1.0,   0.25,   0.5,  -0.5}};

/**
 * 8X like DJI S1000
 *
 *   8 1           cw ccw
 * 7     2       ccw     cw    	X   Z down
 * 6     3        cw     ccw    ^
 *   5 4           ccw cw      	+ > Y
 *
 * columns: X Y Z Roll Pitch Yaw
 * rows: motors 1-8
 */
static double mix_8x[][6] = { \
{0.0,   0.0,  -1.0,  -0.21,   0.50,   0.5},\
{0.0,   0.0,  -1.0,  -0.50,   0.21,  -0.5},\
{0.0,   0.0,  -1.0,  -0.50,  -0.21,   0.5},\
{0.0,   0.0,  -1.0,  -0.21,  -0.50,  -0.5},\
{0.0,   0.0,  -1.0,   0.21,  -0.50,   0.5},\
{0.0,   0.0,  -1.0,   0.50,  -0.21,  -0.5},\
{0.0,   0.0,  -1.0,   0.50,   0.21,   0.5},\
{0.0,   0.0,  -1.0,   0.21,   0.50,  -0.5}};

/**
 * 6D0F control for rotorbits platform
 *
 *  6  1       cw ccw      X
 * 5    2    ccw     cw    ^
 *  4  3       cw ccw      + > Y
 *
 * columns: X Y Z Roll Pitch Yaw
 * rows: motors 1-6
 */
static double mix_6dof_rotorbits[][6] = { \
{-0.2736,    0.3638,   -1.0000,   -0.2293,    0.3921,    0.3443},\
{ 0.6362,    0.0186,   -1.0000,   -0.3638,   -0.0297,   -0.3638},\
{-0.3382,   -0.3533,   -1.0000,   -0.3320,   -0.3638,    0.3546},\
{-0.3382,    0.3533,   -1.0000,    0.3320,   -0.3638,   -0.3546},\
{ 0.6362,   -0.0186,   -1.0000,    0.3638,   -0.0297,    0.3638},\
{-0.2736,   -0.3638,   -1.0000,    0.2293,    0.3921,   -0.3443}};


/**
 * 6D0F control for 5-inch nylon monocoque
 *
 *  6  1       cw ccw      X
 * 5    2    ccw     cw    ^
 *  4  3       cw ccw      + > Y
 *
 * columns: X Y Z Roll Pitch Yaw
 * rows: motors 1-6
 */
static double mix_6dof_5inch_monocoque[][6] = { \
{-0.2296,    0.2296,   -1.0000,   -0.2289,    0.2296,    0.2221},\
{ 0.4742,    0.0000,   -1.0000,   -0.2296,   -0.0000,   -0.2296},\
{-0.2296,   -0.2296,   -1.0000,   -0.2289,   -0.2296,    0.2221},\
{-0.2296,    0.2296,   -1.0000,    0.2289,   -0.2296,   -0.2221},\
{ 0.4742,   -0.0000,   -1.0000,    0.2296,   -0.0000,    0.2296},\
{-0.2296,   -0.2296,   -1.0000,    0.2289,    0.2296,   -0.2221}};


static double (*mix_matrix)[6];
static int initialized;
static int rotors;
static int dof;


int mix_init(rotor_layout_t layout)
{
	switch(layout){
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
	case LAYOUT_6DOF_ROTORBITS:
		rotors = 6;
		dof = 6;
		mix_matrix = mix_6dof_rotorbits;
		break;
	case LAYOUT_6DOF_5INCH_MONOCOQUE:
		rotors = 6;
		dof = 6;
		mix_matrix = mix_6dof_5inch_monocoque;
		break;
	default:
		fprintf(stderr,"ERROR in mix_init() unknown rotor layout\n");
		return -1;
	}

	initialized = 1;
	return 0;
}


int mix_all_controls(double u[6], double* mot)
{
	int i,j;
	if(initialized!=1){
		fprintf(stderr,"ERROR in mix_all_controls, mixing matrix not set yet\n");
		return -1;
	}
	// sum control inputs
	for(i=0;i<rotors;i++){
		mot[i]=0.0;
		for(j=0;j<6;j++){
			mot[i]+=mix_matrix[i][j]*u[j];
		}
	}
	// ensure saturation, should not need to do this if mix_check_saturation
	// was used properly, but here for safety anyway.
	for(i=0;i<rotors;i++){
		if(mot[i]>1.0) mot[i]=1.0;
		else if(mot[i]<0.0) mot[i]=0.0;
	}
	return 0;
}


int mix_check_saturation(int ch, double* mot, double* min, double* max)
{
	int i, min_ch;
	double tmp;
	double new_max = DBL_MAX;
	double new_min = -DBL_MAX;

	if(initialized!=1){
		fprintf(stderr,"ERROR: in check_channel_saturation, mix matrix not set yet\n");
		return -1;
	}

	switch(dof){
	case 4:
		min_ch = 2;
		break;
	case 6:
		min_ch = 0;
		break;
	default:
		fprintf(stderr,"ERROR: in check_channel_saturation, dof should be 4 or 6, currently %d\n", dof);
		return -1;
	}

	if(ch<min_ch || ch>=6){
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


int mix_add_input(double u, int ch, double* mot)
{
	int i;
	int min_ch;

	if(initialized!=1 || dof==0){
		fprintf(stderr,"ERROR: in mix_add_input, mix matrix not set yet\n");
		return -1;
	}
	switch(dof){
	case 4:
		min_ch = 2;
		break;
	case 6:
		min_ch = 0;
		break;
	default:
		fprintf(stderr,"ERROR: in mix_add_input, dof should be 4 or 6, currently %d\n", dof);
		return -1;
	}

	if(ch<min_ch || ch>=6){
		fprintf(stderr,"ERROR: in mix_add_input, ch out of bounds\n");
		return -1;
	}

	// add inputs
	for(i=0;i<rotors;i++){
		mot[i] += u*mix_matrix[i][ch];
		// ensure saturation, should not need to do this if mix_check_saturation
		// was used properly, but here for safety anyway.
		if(mot[i]>1.0) mot[i]=1.0;
		else if(mot[i]<0.0) mot[i]=0.0;
	}
	return 0;
}


