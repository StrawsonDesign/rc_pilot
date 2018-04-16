/**
 * @headerfile fly_types.h
 *
 * structs and enums shared throughout most of the codebase
 */

#ifndef FLY_TYPES_H
#define FLY_TYPES_H

#include <stdint.h>
#include <rc/mpu.h>
#include <mixing_matrix_defs.h>
#include <thrust_map_defs.h>

/**
 * @brief      ARMED or DISARMED to indicate if the feedback controller is
 *             allowed to output to the motors
 */
typedef enum arm_state_t{
	DISARMED,
	ARMED
} arm_state_t;


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
 * The user may elect to power the BBB off the 3-pin JST balance plug or the DC
 * barrel jack. This mode is set in the json config file.
 */
typedef enum battery_connection_t{
	BALANCE_PLUG,
	DC_BARREL_JACK
} battery_connection_t;


/**
 * Setpoint for the feedback controllers. This is written by setpoint_manager
 * and primarily read in by fly_controller. May also be read by printf_manager
 * and log_manager for telemetry
 */
typedef struct setpoint_t{
	arm_state_t arm_state;
	int en_alt_ctrl;	// set to 1 to enable altitude feedback.
	int en_6dof;		// enable direct XY control via 6DOF model
	int en_rpy_ctrl;	// enable the roll pitch yaw controllers

	// direct user inputs
	float Z_throttle;	// only used with direct_throttle user mode
	float X_throttle;	// only used with 6DOF user modes
	float Y_throttle;	// only used with 6DOF user modes

	// attitude setpoint
	float altitude;		// altitude from sea level, positive up (m)
	float altitude_rate;	// desired rate of change in altitude (m/s)
	float roll;		// roll angle (positive tip right) (rad)
	float pitch;		// pitch angle (positive tip back) (rad)
	float yaw;		// yaw angle to magnetive field (rad)
	float yaw_rate;		// desired rate of change in yaw rad/s
} setpoint_t;


/**
 * This is the Core state of the feedback loop. contains most recent values
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
 * Represents current command by the user. This is populated by the
 * input_manager thread which decides to read from mavlink or DSM depending on
 * what it is receiving.
 */
typedef struct user_input_t{
	int user_input_active;		// set to 1 if continuous user input is working
	flight_mode_t flight_mode;	// this is the user commanded flight_mode.
	arm_state_t kill_switch;	// kill motors if set to DISARMED

	// All sticks scaled from -1 to 1
	float thr_stick;		// positive forward
	float yaw_stick;		// positive to the right, CW yaw
	float roll_stick;		// positive to the right
	float pitch_stick;		// positive forward
} user_input_t;


/**
 * Configuration settings read from the json settings file and passed to most
 * threads as they initialize.
 */
typedef struct fly_settings_t{
	// physical parameters
	int num_rotors;
	rotor_layout_t layout;
	int dof;
	thrust_map_t thrust_map;
	rc_mpu_orientation_t orientation;
	float v_nominal;
	battery_connection_t battery_connection;
	int feedback_hz;

	// features
	int enable_freefall_detect;
	int enable_logging;

	// flight modes
	flight_mode_t flight_mode_1;
	flight_mode_t flight_mode_2;
	flight_mode_t flight_mode_3;
	int num_dsm_modes;

	// dsm radio config
	int dsm_thr_ch;
	int dsm_roll_ch;
	int dsm_pitch_ch;
	int dsm_yaw_ch;
	int dsm_mode_ch;
	int dsm_kill_ch;
	int dsm_thr_pol;
	int dsm_roll_pol;
	int dsm_pitch_pol;
	int dsm_yaw_pol;
	int dsm_mode_pol;
	int dsm_kill_pol;

	// printf settings
	int printf_arm;
	int printf_altitude;
	int printf_rpy;
	int printf_sticks;
	int printf_setpoint;
	int printf_u;
	int printf_motors;
	int printf_mode;

} fly_settings_t;




#endif // FLY_TYPES_H
