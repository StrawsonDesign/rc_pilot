/**
 * <include/mix.h>
 *
 * @brief      Functions to mix orthogonal inputs to motor controls
 *
 *             MultiRotors are controlled by mixing roll, pitch, yaw, and
 *             throttle control oututs, a linear combination of which forms the
 *             control output to each motor. The coefficients to this
 *             combination is stored in a mixing matrix based on rotor layout.
 *             Also included here are functions to parse configuration strings
 *             and do the actual mixing.
 */

#ifndef MIXING_MATRIX_H
#define MIXING_MATRIX_H

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
	LAYOUT_6DOF_5INCH_MONOCOQUE
} rotor_layout_t;

/**
 * @brief      Initiallizes the mixing matrix for a given input layout.
 *
 *             For a given number of rotors, layout character ('X','+') and
 *             degrees of freedom in control input (4 or 6), this selects the
 *             correct predefined mixing matrix from mixing_matrix_defs.h. The
 *             matrix is kept locally in mixing _matrix.c to prevent accidental
 *             misuse or modification. Use the other function from
 *             mixing_matrix.c below to interface with it. Used in
 *             mixing_matrix.c
 *
 * @param[in]  layout  The layout enum
 *
 * @return     0 on success, -1 on failure
 */
int mix_init(rotor_layout_t layout);

/**
 * @brief      Fills the vector mot with the linear combination of XYZ, roll
 *             pitch yaw. Not actually used, only for testing.
 *
 *             Fills the vector mot with the linear combination of roll pitch
 *             yaw and throttle based on mixing matrix. If dof = 6, X and Y are
 *             also added. Outputs are blindly saturated between 0 and 1. This
 *             is for rudimentary mixing and testing only. It is recommended to
 *             check for saturation for each input with check_channel_saturation
 *             then add inputs sequentially with add_mixed_input() instead. Used
 *             in mixing_matrix.c
 *
 * @param      u     6 control inputs
 * @param      mot   pointer to motor outputs
 *
 * @return     0 on success, -1 on failure
 */
int mix_all_controls(double u[6], double* mot);

/**
 * @brief      Finds the min and max inputs u that can be applied to a current
 *             set of motor outputs before saturating any one motor.
 *
 *             This is a precurser check to be done before marching a feedback
 *             controller forward so we know what to saturate the transfer
 *             function output at. Outputs min and max are given as pointers
 *             that are written back to. The mot motor array argument is the
 *             array of current motor outputs that the new channel ch will be
 *             adding onto.
 *
 * @param[in]  ch    channel
 * @param[in]      mot   The array of motor signals to be added onto
 * @param[out]      min   The minimum possible input without saturation
 * @param[out]      max   The maximum possible input without saturation
 *
 * @return     0 on success, -1 on failure
 */
int mix_check_saturation(int ch, double* mot, double* min, double* max);

/**
 * @brief      Mixes the control input u for a single channel ch to the existing
 *             motor array mot.
 *
 *             Mixes the control input u for a single channel ch corresponding
 *             to throttle roll pitch etc to the existing motor array mot. No
 *             saturation is done, the input u should be checked for saturation
 *             validity with check_channel_saturation() first. Used in
 *             mixing_matrix.c
 *
 * @param[in]  u     control input
 * @param[in]  ch    channel
 * @param      mot   array of motor channels
 *
 * @return     0 on success, -1 on failure
 */
int mix_add_input(double u, int ch, double* mot);


#endif // MIXING_MATRIX_H