/**
 * @file mixing_matrix.h
 *
 * @brief      Functions to mix orthogonal inputs to motor controls
 */

#ifndef MIXING_MATRIX_H
#define MIXING_MATRIX_H

#include <mixing_matrix_defs.h>

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
int initialize_mixing_matrix(rotor_layout_t layout);

/**
 * @brief      Fills the vector mot with the linear combination of roll pitch
 *             yaw and throttle based on mixing matrix.
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
 * @param      mot   The mot
 *
 * @return     0 on success, -1 on failure
 */
int mix_all_controls(float u[6], float* mot);

/**
 * @brief      Checks to see if the desired input u on channel ch would saturate
 *             a motor.
 *
 *             Checks to see if the desired input u on channel ch corresponding
 *             to throttle, roll, pitch, etc would saturate a motor in the given
 *             motor array mot. If saturation would occur, it returns the
 *             maximum allowed input u. Used in mixing_matrix.c
 *
 * @param[in]  ch    channel
 * @param      mot   The motor signal to be saturated
 * @param      min   The minimum
 * @param      max   The maximum
 *
 * @return     maximum allowed input u if saturation occurs. -1 if saturation
 *             does not occur. !!!
 */
int check_channel_saturation(int ch, float* mot, float* min, float* max);

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
int add_mixed_input(float u, int ch, float* mot);


#endif // MIXING_MATRIX_H