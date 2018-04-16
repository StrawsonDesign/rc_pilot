/**
 * @file feedback.h
 *
 * @brief      Functions to start and stop the feedback controller ISR
 *
 *             Here lies the heart and soul of the operation. I wish the whole
 *             flight controller could be just this file, woe is me.
 *             initialize_controllers() pulls in the control constants from
 *             json_settings and sets up the discrete controllers. From then on
 *             out, feedback_controller() should be called by the IMU interrupt
 *             at feedback_hz until the program is shut down.
 *             feedback_controller() will monitor the setpoint which is
 *             constantly being changed by setpoint_manager(). It also does
 *             state estimation to update core_state() even when the controller
 *             is disarmed. When controllers are enabled or disabled mid-flight
 *             by mode switches then the controllers are started smoothly.
 */

#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <fly_types.h>

/**
 * @brief      This is how outside functions should stop the flight controller.
 *
 *             It would be reasonable to set motors to 0 here, but since this
 *             function can be called from anywhere that might produce
 *             conflicts. Instead the interrupt service routine will do this on
 *             the next loop after disarming to maintain timing of pulses to the
 *             motors
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
int init_controller(cstate_t* cstate, setpoint_t* setpoint, fly_settings_t* settings);

#endif // FEEDBACK_H

