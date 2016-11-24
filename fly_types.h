/*******************************************************************************
* fly_types.h
*
* structs and enums used by the flight controller
*******************************************************************************/

#ifndef FLY_TYPES
#define FLY_TYPES 

/************************************************************************
* arm_state_t
*
* ARMED or DISARMED to indicate if the controller is running
************************************************************************/
typedef enum arm_state_t{
	DISARMED,
	ARMED
} arm_state_t;

/*******************************************************************************
* flight_mode_t
*	
* flight_mode determines how the setpoint manager behaves
*
* DIRECT_THROTTLE: user inputs translate directly to the throttle, 
* roll, pitch, & yaw setpoints. No altitude feedback control.
*	
* ALTITUDE_HOLD: user input translates directly to roll, pitch, & yaw
* rate setpoints. Throttle translates to altitude-rate setpoint and altitude
* is maintained with feedback.
*
* EMERGENCY_LAND: Roll and pitch setpoints are left at 0. Altitude setpoint is
* slowly lowered until land detected.
*******************************************************************************/
typedef enum flight_mode_t{
	DIRECT_THROTTLE,
	TESTING
} flight_mode_t;


/************************************************************************
* layout_t
*
* possible rotor configurations, see mixing_matrix_defs.h
************************************************************************/
typedef enum layout_t{
	LAYOUT_4X,
	LAYOUT_4PLUS,
	LAYOUT_6X,
	LAYOUT_6PLUS,
	LAYOUT_8X,
	LAYOUT_8PLUS,
	LAYOUT_6DOF
} layout_t;


/*******************************************************************************
* setpoint_t
*
* Setpoint for the feedback controllers. This is written by setpoint_manager 
* and read in by fly_controller.
*******************************************************************************/
typedef struct setpoint_t{
	int altitude_ctrl_en;	// set to 1 to enable altitude feedback.
	int en_6dof;			// enable direct XY control via 6DOF model
	
	// direct user inputs
	float Z_throttle;		// only used with direct_throttle user mode
	float X_throttle;		// only used with direct_throttle user mode
	float Y_throttle;		// only used with direct_throttle user mode
	
	// attitude setpoint
	float altitude;			// altitude from sea level, positive up (m)
	float altitude_rate;	// desired rate of change in altitude (m/s)
	float roll;				// roll angle (positive tip right) (rad)
	float pitch;			// pitch angle (positive tip back) (rad)
	float yaw;				// yaw angle to magnetive field (postiive CCW)(rad)
	float yaw_rate;			// yaw_rate in rad/s
} setpoint_t;


/*******************************************************************************
* cstate_t
*
* contains most recent values reported by the fly_controller. Should only be 
* written to by the flight controller after initialization.
*******************************************************************************/
typedef struct cstate_t{
	uint64_t start_time_us;	// time when IMU interrupt routine started
	uint64_t time_us; 		// last time controller has finished a step
	uint64_t step;			// num steps since controller was armed

	// current state orientation and position
	float alt;				// altitude estimate from sea level (m)
	float roll;				// current roll angle (rad)
	float pitch;			// current pitch angle (rad)
	float yaw;				// current yaw angle (rad)

	// current state velocities
	float dAlt;				// first derivative of altitude (m/s)
	float dRoll;			// first derivative of roll (rad/s)
	float dPitch;			// first derivative of pitch (rad/s)
	float dYaw;				// first derivative of yaw (rad/s)

	// misc
	float vbatt;			// main battery pack voltage (v)
} cstate_t;


/*******************************************************************************
* user_input_t
*
* Represents current command by the user. This is populated by the input_manager
* thread
*******************************************************************************/
typedef struct user_input_t{
	int user_input_active;		// set to 1 if continuous user input is working
	flight_mode_t flight_mode;  // this is the user commanded flight_mode. 
	arm_state_t kill_switch; 	// kill motors if set to DISARMED
	
	// All sticks scaled from -1 to 1
	float throttle_stick; 	// positive up
	float yaw_stick;		// positive to the right, CW yaw
	float roll_stick;		// positive to the right
	float pitch_stick;		// positive up
} user_input_t;


/*******************************************************************************
* fly_settings_t
*
* basic settings read from the json settings file
*******************************************************************************/
typedef struct fly_settings_t{
	// physical parameters
	int num_rotors;
	layout_t layout;
	imu_orientation_t bbb_orientation;
	float v_nominal;

	// features
	int enable_freefall_detect;
	int enable_logging;

	// flight modes
	flight_mode_t flight_mode_1;
	flight_mode_t flight_mode_2;
	flight_mode_t flight_mode_3;

	// dsm radio config
	int dsm_throttle_ch;
	int dsm_roll_ch;
	int dsm_pitch_ch;
	int dsm_yaw_ch;
	int dsm_mode_ch;
	int dsm_kill_ch;
	int dsm_throttle_pol;
	int dsm_roll_pol;
	int dsm_pitch_pol;
	int dsm_yaw_pol;
	int dsm_mode_pol;
	int dsm_kill_pol;
	int dsm_num_modes;

	// feedback loop speed
	int feedback_hz
} fly_settings_t;


/*******************************************************************************
* fly_controllers_t
*
* collection of all feedback controllers, used by json_settings.c to pass
* controllers to feedback_controller.c neatly
*******************************************************************************/
typedef struct fly_controllers_t{
	d_filter_t altitude_controller;
	d_filter_t roll_controller;
	d_filter_t pitch_controller;
	d_filter_t yaw_controller;
}fly_controllers_t;

#endif // FLY_TYPES