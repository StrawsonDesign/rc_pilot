/**
 * @file feedback.h
 *
 * @brief      Functions to start and stop the feedback controller ISR
 *
 *             Here lies the heart and soul of the operation. I wish the whole
 *             flight controller could be just this file, woe is me.
 *             initialize_controllers() pulls in the control constants from
 *             json_settings and sets up the discrete controllers. From then on
 *             out, feedback_controller() should be called by the IMU interrupt
 *             at feedback_hz until the program is shut down.
 *             feedback_controller() will monitor the setpoint which is
 *             constantly being changed by setpoint_manager(). It also does
 *             state estimation to update core_state() even when the controller
 *             is disarmed. When controllers are enabled or disabled mid-flight
 *             by mode switches then the controllers are started smoothly.
 */

#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <stdint.h> // for uint64_t
#include <rc_pilot_defs.h>

/**
 * This is the state of the feedback loop. contains most recent values
 * reported by the feedback controller. Should only be written to by the
 * feedback controller after initialization.
 */
typedef struct feedback_state_t{
	int initialized;	///< set to 1 after feedback_init()
	arm_state_t arm_state;	///< actual arm state as reported by feedback controller
	uint64_t arm_time_ns;	///< time since boot when controller was armed
	uint64_t loop_index;	///< increases every time feedback loop runs
	uint64_t last_step_ns;	///< last time controller has finished a step

	double altitude_bmp;	///< altitude estimate using bmp from sea level (m)
	double altitude_kf;	///< altitude estimate using kalman filter
	double alt_kf_vel;	///< z velocity estimate using kalman filter
	double alt_kf_accel;	///< z accel estimate using kalman filter
	double roll;		///< current roll angle (rad)
	double roll_rate;	///< current roll anglular velocity (rad/s)
	double pitch;		///< current pitch angle (rad)
	double pitch_rate;	///< current pitch anglular velocity (rad/s)
	double yaw;		///< current yaw angle (rad)
	double yaw_rate;	///< current yaw anglular velocity (rad/s)
	double v_batt;		///< main battery pack voltage (v)

	double u[6];		///< siso controller outputs
	double m[8];		///< signals sent to motors after mapping
} feedback_state_t;

extern feedback_state_t fstate;

/**
 * @brief      Initial setup of all feedback controllers. Should only be called
 *             once on program start.
 *
 * @param      setpoint  pointer to global setpoint struct
 * @param      settings  pointer to global settings struct
 *
 * @return     0 on success, -1 on failure
 */
int feedback_init();

/**
 * @brief      This is how outside functions should stop the flight controller.
 *
 *             It would be reasonable to set motors to 0 here, but since this
 *             function can be called from anywhere that might produce
 *             conflicts. Instead the interrupt service routine will do this on
 *             the next loop after disarming to maintain timing of pulses to the
 *             motors
 *
 * @return     0 on success, -1 on failure
 */
int feedback_disarm();

/**
 * @brief      This is how outside functions should start the flight controller.
 *
 * @return     0 on success, -1 on failure
 */
int feedback_arm();

int feedback_cleanup();






#endif // FEEDBACK_H

