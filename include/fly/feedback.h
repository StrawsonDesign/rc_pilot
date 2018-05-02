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

/**
 * @brief      ARMED or DISARMED to indicate if the feedback controller is
 *             allowed to output to the motors
 */
typedef enum arm_state_t{
	DISARMED,
	ARMED
} arm_state_t;



/**
 * This is the core state of the feedback loop. contains most recent values
 * reported by the feedback controller. Should only be written to by the
 * feedback controller after initialization.
 */
typedef struct cstate_t{
	uint64_t loop_index;
	uint64_t last_step_us;	// last time controller has finished a step

	float altitude;		// altitude estimate from sea level (m)
	float roll;		// current roll angle (rad)
	float pitch;		// current pitch angle (rad)
	float yaw;		// current yaw angle (rad)
	float v_batt;		// main battery pack voltage (v)

	float u[6];		// siso controller outputs
	float m[8];		// signals sent to motors after mapping
} cstate_t;


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

/**
 * @brief      Gets the arm state.of the feedback controller
 *
 *             Returns the arm state of the feedback controller so outside
 *             functions, namely the setpoint_manager, can tell if the feedback
 *             controller is armed or not.
 *
 * @return     The controller arm state.
 */
arm_state_t feedback_get_arm_state();

/**
 * @brief      gets a copy of the current feedback controller state
 *
 *             mostly used by printf_manager to print status
 *
 * @return     copy of the current cstate struct
 */
cstate_t feedback_get_cstate();



#endif // FEEDBACK_H

