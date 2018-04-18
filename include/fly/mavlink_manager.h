/**
 * <mavlink_manager.h>
 *
 * Functions to start and stop the mavlink manager
 */

#ifndef MAVLINK_MANAGER_H
#define MAVLINK_MANAGER_H



/**
 * @brief      Starts the mavlink manager
 *
 * @return     0 on success, -1 on failure
 */
int start_mavlink_manager();

/**
 * @brief      stops the mavlink manager
 *
 * @return     0 if thread exited cleanly, -1 if exit timed out.
 */
int cleanup_mavlink_manager();

#endif // MAVLINK_MANAGER_H
