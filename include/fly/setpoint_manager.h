/**
 * <fly/setpoint_manager.h>
 *
 * @brief      Functions to start and stop the setpoint manager thread
 */

#ifndef SETPOINT_MANAGER_H
#define SETPOINT_MANAGER_H

/**
 * flight_mode_t determines how the setpoint manager behaves
 *
 * DIRECT_THROTTLE: user inputs translate directly to the throttle, roll, pitch,
 * & yaw setpoints. No altitude feedback control. On 6DOF platforms roll and
 * pitch are kept level and right joystick inputs are direct to left/right
 * forward/back thrust
 *
 * FALLBACK_4DOF: only applicable to 6DOF platforms. Ignores left/right and
 * forward/back inputs, user controls pitch and roll instead.
 */
typedef enum flight_mode_t{
	DIRECT_THROTTLE,
	FALLBACK_4DOF,
	TEST_BENCH
} flight_mode_t;

/**
 * @brief      Starts the setpoint manager thread.
 *
 * @return     0 on success, -1 on failure
 */
int start_setpoint_manager();

/**
 * @brief      Waits for the setpoint manager thread to exit.
 *
 * @return     0 on clean exit, -1 if exit timed out
 */
int join_setpoint_manager_thread();

#endif // SETPOINT_MANAGER_H
