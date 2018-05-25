/**
 * <setpoint_manager.h>
 *
 * @brief      Functions to start and stop the setpoint manager thread
 */

#ifndef SETPOINT_MANAGER_H
#define SETPOINT_MANAGER_H

#include <fly/feedback.h>

/**
 * Setpoint for the feedback controllers. This is written by setpoint_manager
 * and primarily read in by fly_controller. May also be read by printf_manager
 * and log_manager for telemetry
 */
typedef struct setpoint_t{
	arm_state_t arm_state;
	int en_alt_ctrl;	///< enable altitude feedback.
	int en_6dof;		///< enable direct XY control via 6DOF model
	int en_rpy_ctrl;	///< enable the roll pitch yaw controllers

	// direct user inputs
	float Z_throttle;	///< used only when altitude controller disabled
	float X_throttle;	///< only used when 6dof is enabled
	float Y_throttle;	///< only used when 6dof is enabled
	float Roll_throttle;	///< only used when roll_pitch_yaw controllers are disbaled
	float Pitch_throttle;	///< only used when roll_pitch_yaw controllers are disbaled
	float Yaw_throttle;	///< only used when roll_pitch_yaw controllers are disbaled

	// attitude setpoint
	float altitude;		///< altitude from sea level, positive up (m)
	float altitude_rate;	///< desired rate of change in altitude (m/s)
	float roll;		///< roll angle (positive tip right) (rad)
	float pitch;		///< pitch angle (positive tip back) (rad)
	float yaw;		///< yaw angle to magnetive field (rad)
	float yaw_rate;		///< desired rate of change in yaw rad/s
} setpoint_t;

/**
 * @brief      Starts the setpoint manager thread.
 *
 * @return     0 on success, -1 on failure
 */
int setpoint_manager_init();

/**
 * @brief      fetches a copy of the current setpoint
 *
 * @return     copy of the current setpoint
 */
setpoint_t setpoint_manager_get_sp();

/**
 * @brief      Waits for the setpoint manager thread to exit.
 *
 * @return     0 on clean exit, -1 if exit timed out
 */
int setpoint_manager_cleanup();


#endif // SETPOINT_MANAGER_H
