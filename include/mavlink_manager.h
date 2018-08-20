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
int mavlink_manager_init(void);

/**
 * @brief      stops the mavlink manager
 *
 * @return     0 if thread exited cleanly, -1 if exit timed out.
 */
int mavlink_manager_cleanup(void);

#endif // MAVLINK_MANAGER_H
