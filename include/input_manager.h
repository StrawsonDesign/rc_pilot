/**
 * @file input_manager.h
 *
 * @brief      Functions to start and stop the input manager thread which is the
 *             translation beween control inputs from mavlink and DSM to the
 *             user_input struct which is read by the setpoint manager.
 */

#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H

#include <fly_types.h>

/**
 * @brief      Starts an input manager thread.
 *
 *             Watch for new DSM data and translate into local user mode. Used
 *             in input_manager.c
 *
 * @param      user_input  pointer to user_intput_t struct, command input
 * @param      settings    pointer to fly_settings_t sruct, json file config settings
 *
 * @return     0 on success, -1 on failure
 */
int start_input_manager(user_input_t* user_input, fly_settings_t* settings);

/**
 * @brief      Waits for the input manager thread to exit
 *
 *             This should only be called after the program flow state is set to
 *             EXITING as that's the only thing that will cause the thread to
 *             exit on its own safely. Used in input_manager.c
 *
 * @return     0 on clean exit, -1 if exit timed out.
 */
int join_input_manager_thread();

#endif // INPUT_MANAGER_H
