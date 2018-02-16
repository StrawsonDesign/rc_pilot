/**
 * @headerfile printf_manager.h
 *
 * @brief      Functions to start and stop the printf mnaager which is a
 *             separate thread printing data to the console for debugging.
 */


#ifndef PRINTF_MANAGER_H
#define PRINTF_MANAGER_H

#include <fly_types.h>

/**
 * @brief      Start the printf_manager thread which should be the only thing
 *             printing to the screen besides error messages from other threads.
 *
 *
 * @param      cstate      The cstate
 * @param      setpoint    The setpoint
 * @param      user_input  The user input
 * @param      settings    The settings
 *
 * @return     0 on success, -1 on failure
 */
int start_printf_manager(cstate_t* cstate, setpoint_t* setpoint, \
			user_input_t* user_input, fly_settings_t* settings);

/**
 * @brief      Waits for the printf manager thread to exit.
 *
 *             Used in printf_manager.c
 *
 * @return     0 on clean exit, -1 on exit time out/force close
 */
int join_printf_manager_thread();

#endif //PRINTF_MANAGER_H