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
 * @param      setpoint    pointer to the setpoint_t struct
 * @param      user_input  pointer to the user_input_t struct, user input command
 * @param      cstate      pointer to the cstate_t struct, feedback controller core struct
 * @param      settings    pointer to the fly_settings_t struct, setting configs from the json file.
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
