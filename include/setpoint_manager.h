/**
 * @file setpoint_manager.h
 *
 * @brief      Functions to start and stop the setpoint manager thread
 */

#ifndef SETPOINT_MANAGER_H
#define SETPOINT_MANAGER_H

#include <fly_types.h>

/**
 * @brief      Starts the setpoint manager thread.
 *
 *             Used in setpoint_manager.c
 *
 * @param      setpoint    The setpoint
 * @param      user_input  The user input
 * @param      cstate      The cstate
 * @param      settings    The settings
 *
 * @return     0 on success, -1 on failure
 */
int start_setpoint_manager(setpoint_t* setpoint, user_input_t* user_input,
				cstate_t* cstate, fly_settings_t* settings);

/**
 * @brief      Waits for the setpoint manager thread to exit.
 *
 *             Used in setpoint_manager.c
 *
 * @return     0 on clean exit, -1 if exit timed out
 */
int join_setpoint_manager_thread();

#endif // SETPOINT_MANAGER_H