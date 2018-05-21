/**
 * <fly/battery_manager.h>
 *
 * @brief Functions to start and stop the battery manager thread
 *
 */

#ifndef BATTERY_MANAGER_H
#define BATTERY_MANAGER_H

/**
 * The user may elect to power the BBB off the 3-pin JST balance plug or the DC
 * barrel jack. This mode is set in the json config file.
 */
typedef enum battery_connection_t{
	BALANCE_PLUG,
	DC_BARREL_JACK
} battery_connection_t;

/**
 * @brief      Starts a battery manager thread.
 *
 * @return     0 on success, -1 on failure
 */
int battery_manager_init();

/**
 * @brief      Waits for the battery manager thread to exit.
 *
 *             This should only be called after the program flow state is set to
 *             EXITING as that's the only thing that will cause the thread to
 *             exit on its own safely. Used in battery_manager.c
 *
 * @return     0 if thread exited cleanly, -1 if exit timed out.
 */
int battery_manager_cleanup();

#endif // BATTERY_MANAGER_H
