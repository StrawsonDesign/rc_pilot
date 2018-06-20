/**
 * <setpoint_manager.h>
 *
 * @brief      Setpoint manager runs at the same rate as the feedback controller
 *             and is the interface between the user inputs (input manager) and
 *             the feedback controller setpoint. currently it contains very
 *             simply logic and runs very quickly which is why it's okay to run
 *             in the feedback ISR right before the feedback controller. In the
 *             future this is where go-home and other higher level autonomy will
 *             live.
 *
 *             This serves to allow the feedback controller to be as simple and
 *             clean as possible by putting all high-level manipulation of the
 *             setpoints here. Then feedback-controller only needs to march the
 *             filters and zero them out when arming or enabling controllers
 */

#ifndef SETPOINT_MANAGER_H
#define SETPOINT_MANAGER_H

#include <rc_pilot_defs.h>

/**
 * Setpoint for the feedback controllers. This is written by setpoint_manager
 * and primarily read in by fly_controller. May also be read by printf_manager
 * and log_manager for telemetry
 */
typedef struct setpoint_t{
	int initialized;	///< set to 1 once setpoint manager has initialized
	int en_alt_ctrl;	///< enable altitude feedback.
	int en_rpy_ctrl;	///< enable the roll pitch yaw controllers
	int en_6dof;		///< enable direct XY control via 6DOF model

	// direct passthrough user inputs to mixing matrix
	double Z_throttle;	///< used only when altitude controller disabled
	double X_throttle;	///< only used when 6dof is enabled, positive forward
	double Y_throttle;	///< only used when 6dof is enabled, positive right
	double roll_throttle;	///< only used when roll_pitch_yaw controllers are disbaled
	double pitch_throttle;	///< only used when roll_pitch_yaw controllers are disbaled
	double yaw_throttle;	///< only used when roll_pitch_yaw controllers are disbaled

	// attitude setpoint
	double altitude;	///< altitude from sea level, positive up (m)
	double altitude_rate;	///< desired rate of change in altitude (m/s)
	double roll;		///< roll angle (positive tip right) (rad)
	double pitch;		///< pitch angle (positive tip back) (rad)
	double yaw;		///< glabal yaw angle, positive left
	double yaw_rate;	///< desired rate of change in yaw rad/s
} setpoint_t;

extern setpoint_t setpoint;

/**
 * @brief      Initializes the setpoint manager.
 *
 * @return     0 on success, -1 on failure
 */
int setpoint_manager_init();

/**
 * @brief      updates the setpoint manager, call this before feedback loop
 *
 * @return     0 on success, -1 on failure
 */
int setpoint_manager_update();

/**
 * @brief      cleans up the setpoint manager, not really necessary but here for
 *             completeness
 *
 * @return     0 on clean exit, -1 if exit timed out
 */
int setpoint_manager_cleanup();


#endif // SETPOINT_MANAGER_H
