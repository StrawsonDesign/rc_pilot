/**
 * @file battery_manager.h
 *
 * @brief Functions to start and stop the battery manager thread
 *
 */

#ifndef BATTERY_MANAGER_H
#define BATTERY_MANAGER_H

#include <fly_types.h>


/**
 * @brief      Starts a battery manager thread.
 *
 *             Used in battery_manager.c
 *
 * @param      core_state    pointer to the most recent attitude and controller values
 *                           reported by the fly_controller
 * @param      fly_settings  pointer to flight config settings from the json file
 *
 * @return     0 on success, -1 on failure
 */
int start_battery_manager(cstate_t* core_state, fly_settings_t* fly_settings);

/**
 * @brief      Waits for the battery manager thread to exit.
 *
 *             This should only be called after the program flow state is set to
 *             EXITING as that's the only thing that will cause the thread to
 *             exit on its own safely. Used in battery_manager.c
 *
 * @return     0 if thread exited cleanly, -1 if exit timed out.
 */
int join_battery_manager_thread();

#endif // BATTERY_MANAGER_H
