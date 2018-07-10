/**
 * <rc_pilot_defs.h>
 *
 * @brief constants and parameters
 */

#ifndef RC_PILOT_DEFS_H
#define RC_PILOT_DEFS_H

/**
 * @brief      ARMED or DISARMED to indicate if the feedback controller is
 *             allowed to output to the motors
 */
typedef enum arm_state_t{
	DISARMED,
	ARMED
} arm_state_t;

// Flight Core Constants
#define ARM_TIP_THRESHOLD	0.2	///< radians from level to allow arming sequence
#define TIP_ANGLE		1.5	///< radiands of roll or pitch to consider tipped over

// order of control inputs
// throttle(Z), roll, pitch, YAW, sideways (X),forward(Y)
#define VEC_X			0
#define VEC_Y			1
#define VEC_Z			2
#define VEC_ROLL		3
#define VEC_PITCH		4
#define VEC_YAW			5

// user control parameters
#define MAX_YAW_RATE		2.5	// rad/s
#define MAX_ROLL_SETPOINT	0.4	// rad
#define MAX_PITCH_SETPOINT	0.4	// rad
#define MAX_CLIMB_RATE		1.0	// m/s
#define YAW_DEADZONE		0.02
#define THROTTLE_DEADZONE	0.02
#define SOFT_START_SECONDS	1.0	// controller soft start seconds

// controller absolute limits
#define MAX_ROLL_COMPONENT	1.0
#define MAX_PITCH_COMPONENT	1.0
#define MAX_YAW_COMPONENT	1.0

#define MAX_X_COMPONENT		1.0
#define MAX_Y_COMPONENT		1.0

#define MOTOR_IDLE_CMD      0.05
#define MIN_Z_COMPONENT		0.0
#define MAX_Z_COMPONENT		1.0

// Files
#define LOG_DIR			"/home/debian/rc_pilot_logs/"
#define SETTINGS_FILE	"/home/debian/rc_pilot/settings/rc_pilot_settings.json"


// for future modes, not used yet
#define LAND_TIMEOUT		0.3
#define DISARM_TIMEOUT		4.0

#endif // RC_PILOT_DEFS_H