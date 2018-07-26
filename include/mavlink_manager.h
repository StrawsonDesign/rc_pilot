/**
 * <mavlink_manager.h>
 *
 * Functions to start and stop the mavlink manager
 */

#ifndef MAVLINK_MANAGER_H
#define MAVLINK_MANAGER_H

/**
 * this is an external struct declared here and can be read by anything that
 * includes <mavlink_manager.h>
 *
 * It is populated by mavlink_manager except in the case where state_estimator
 * determines the data is too old which is roughly 4 cycles of the feedback
 * controller. When either mavlink manager or state_estimator switches is_active
 * from 1 to 0, the position data is left as-is so the last position value can
 * be read by other modules.
 */
typedef struct mocap_state_t{
	/**
	 * is_active is set to 1 after first packet, reset to 0 by mavlink_manager as soon as
	 * connection is lost. State estimator may set this to 0 too if the
	 * timestamp is deemed too old.
	 */
	int is_active;
	uint64_t timestamp_ns;	///< rc_nanos_since_boot() of last packet
	double q[4];		///< quaternion from mocap
	double tait_bryan[3];	///< derived from the quaternion for convenience
	double position[3];	///< X Y Z position in meters. NED coordinates
}
mocap_state_t;

/**
 * settings are external, so just include this header and read from it
 */
mocap_state_t mocap_state;

/**
 * @brief      Starts the mavlink manager
 *
 * @return     0 on success, -1 on failure
 */
int mavlink_manager_init();

/**
 * @brief      stops the mavlink manager
 *
 * @return     0 if thread exited cleanly, -1 if exit timed out.
 */
int mavlink_manager_cleanup();

#endif // MAVLINK_MANAGER_H
