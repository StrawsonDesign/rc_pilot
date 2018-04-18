/**
 * @headerfile mixing_matrix_defs.h <mixing_matrix_defs.h>
 *
 * matrix definitions for mixing_matrix.c
 *
 *                rotor
 *          X Y Z  Roll Pitch Yaw
 *       1
 *       2
 *       3
 *       4
 *       5
 *       6
 *
 * Z points down so positive yaw is turning right from 1st person perspective
 */

#ifndef MIXING_MATRIX_DEFS_H
#define MIXING_MATRIX_DEFS_H

#define MAX_INPUTS 6	///< up to 6 control inputs (roll,pitch,yaw,z,x,y)
#define MAX_ROTORS 8	///< up to 8 rotors

/**
 * @brief enum for possible mixing matrices defined here
 *
 * possible rotor configurations, see mixing_matrix_defs.h
 */
typedef enum rotor_layout_t{
	LAYOUT_4X,
	LAYOUT_4PLUS,
	LAYOUT_6X,
	LAYOUT_8X,
	LAYOUT_6DOF_ROTORBITS,
	LAYOUT_6DOF_4INCH_MONOCOQUE,
	LAYOUT_6DOF_5INCH_MONOCOQUE,
} rotor_layout_t;

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
float mix_4x[][6] = {\
{0.0,   0.0,  -1.0,  -0.5,   0.5,   0.5},\
{0.0,   0.0,  -1.0,  -0.5,  -0.5,  -0.5},\
{0.0,   0.0,  -1.0,   0.5,  -0.5,   0.5},\
{0.0,   0.0,  -1.0,   0.5,   0.5,  -0.5}\
};

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
float mix_4plus[][6] = {\
{0.0,   0.0,  -1.0,   0.0,   0.5,   0.5},\
{0.0,   0.0,  -1.0,  -0.5,   0.0,  -0.5},\
{0.0,   0.0,  -1.0,   0.0,  -0.5,   0.5},\
{0.0,   0.0,  -1.0,   0.5,   0.0,  -0.5}\
};

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
float mix_6x[][6] = {\
{0.0,   0.0,  -1.0,  -0.25,   0.5,   0.5},\
{0.0,   0.0,  -1.0,  -0.50,   0.0,  -0.5},\
{0.0,   0.0,  -1.0,  -0.25,  -0.5,   0.5},\
{0.0,   0.0,  -1.0,   0.25,  -0.5,  -0.5},\
{0.0,   0.0,  -1.0,   0.50,   0.0,   0.5},\
{0.0,   0.0,  -1.0,   0.25,   0.5,  -0.5}\
};

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
float mix_8x[][6] = {\
{0.0,   0.0,  -1.0,  -0.21,   0.50,   0.5},\
{0.0,   0.0,  -1.0,  -0.50,   0.21,  -0.5},\
{0.0,   0.0,  -1.0,  -0.50,  -0.21,   0.5},\
{0.0,   0.0,  -1.0,  -0.21,  -0.50,  -0.5},\
{0.0,   0.0,  -1.0,   0.21,  -0.50,   0.5},\
{0.0,   0.0,  -1.0,   0.50,  -0.21,  -0.5},\
{0.0,   0.0,  -1.0,   0.50,   0.21,   0.5},\
{0.0,   0.0,  -1.0,   0.21,   0.50,  -0.5}\
};

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
float mix_rotorbits_6dof[][6] = {\
{-0.2736,    0.3638,   -1.0000,   -0.2293,    0.3921,    0.3443},\
{ 0.6362,    0.0186,   -1.0000,   -0.3638,   -0.0297,   -0.3638},\
{-0.3382,   -0.3533,   -1.0000,   -0.3320,   -0.3638,    0.3546},\
{-0.3382,    0.3533,   -1.0000,    0.3320,   -0.3638,   -0.3546},\
{ 0.6362,   -0.0186,   -1.0000,    0.3638,   -0.0297,    0.3638},\
{-0.2736,   -0.3638,   -1.0000,    0.2293,    0.3921,   -0.3443}\
};

/**
 * 6D0F control for 4-inch nylon monocoque
 *
 *  6  1       cw ccw      X
 * 5    2    ccw     cw    ^
 *  4  3       cw ccw      + > Y
 *
 * columns: X Y Z Roll Pitch Yaw
 * rows: motors 1-6
 */
float mix_4inch_monocoque[][6] = {\
{-0.2736,    0.3638,   -1.0000,   -0.2293,    0.3921,    0.3443},\
{ 0.6362,    0.0186,   -1.0000,   -0.3638,   -0.0297,   -0.3638},\
{-0.3382,   -0.3533,   -1.0000,   -0.3320,   -0.3638,    0.3546},\
{-0.3382,    0.3533,   -1.0000,    0.3320,   -0.3638,   -0.3546},\
{ 0.6362,   -0.0186,   -1.0000,    0.3638,   -0.0297,    0.3638},\
{-0.2736,   -0.3638,   -1.0000,    0.2293,    0.3921,   -0.3443}\
};

/**
 * 6D0F control for 5-inch PETG monocoque
 *
 *  6  1       cw ccw      X
 * 5    2    ccw     cw    ^
 *  4  3       cw ccw      + > Y
 *
 * columns: X Y Z Roll Pitch Yaw
 * rows: motors 1-6
 */
float mix_5inch_monocoque[][6] = {\
{-0.2736,    0.3638,   -1.0000,   -0.2293,    0.3921,    0.3443},\
{ 0.6362,    0.0186,   -1.0000,   -0.3638,   -0.0297,   -0.3638},\
{-0.3382,   -0.3533,   -1.0000,   -0.3320,   -0.3638,    0.3546},\
{-0.3382,    0.3533,   -1.0000,    0.3320,   -0.3638,   -0.3546},\
{ 0.6362,   -0.0186,   -1.0000,    0.3638,   -0.0297,    0.3638},\
{-0.2736,   -0.3638,   -1.0000,    0.2293,    0.3921,   -0.3443}\
};

#endif  //MIXING_MATRIX_DEFS_H