/**
 * @file feedback.h
 *
 * @brief Functions to start and stop the feedback controller ISR
 *
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <fly_types.h>

/**
 * @brief      This is how outside functions should stop the flight controller.
 *
 *
 * @return     0 on success, -1 on failure
 */
int disarm_controller();

/**
 * @brief      This is how outside functions should start the flight controller.
 *
 * @return     0 on success, -1 on failure
 */
int arm_controller();

/**
 * @brief      Gets the controller arm state.
 *
 *             Returns the arm state of the controller so outside functions,
 *             namely the setpoint_manager, can tell if the controller is armed
 *             or not. Used in feedback_controller.c
 *
 * @return     The controller arm state.
 */
arm_state_t get_controller_arm_state();

/**
 * @brief      Initial setup of all feedback controllers. Should only be called
 *             once on program start.
 *
 * @param      cstate    pointer to global system core state
 * @param      setpoint  pointer to global setpoint struct
 * @param      settings  pointer to global settings struct
 *
 * @return     0 on success, -1 on failure
 */
int initialize_controller(cstate_t* cstate, setpoint_t* setpoint, fly_settings_t* settings);

#endif // CONTROLLER_H

