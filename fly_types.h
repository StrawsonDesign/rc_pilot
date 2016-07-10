/*******************************************************************************
* fly_types.h
*
* structs and enums used by the flight controller
*******************************************************************************/

#ifndef FLY_TYPES
#define FLY_TYPES 

/************************************************************************
* 	arm_state_t
*
*	ARMED or DISARMED to indicate if the controller is running
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
* ATTITUDE_DIRECT_THROTTLE: user inputs translate directly to the throttle, 
* roll, pitch, & yaw setpoints. No altitude feedback control.
*	
* ATTITUDE_ALTITUDE_HOLD: user input translates directly to roll, pitch, & yaw
* rate setpoints. Throttle translates to altitude-rate setpoint and altitude
* is maintained with feedback.
*
* 6DOF_CONTROL: Roll an pitch setpoints are left at 0. User instead controls 
* position with direct inputs to X and Y thrust.
*
* EMERGENCY_LAND: Roll and pitch setpoints are left at 0. Altitude setpoint is
* slowly lowered until land detected.
*******************************************************************************/
typedef enum flight_mode_t{
	ATTITUDE_DIRECT_THROTTLE,
	ATTITUDE_ALTITUDE_HOLD,
	6DOF_CONTROL,
	EMERGENCY_LAND
} flight_mode_t;


/*******************************************************************************
* setpoint_t
*
* Setpoint for the feedback controllers. This is written by setpoint_manager 
* and read in by fly_controller.
*******************************************************************************/
typedef struct setpoint_t{
	arm_state_t arm_state;	// see arm_state_t declaration

	int altitude_ctrl_en;	// set to 1 to enable altitude feedback.
	int 6dof_en;			// enable direct XY control via 6DOF model
	
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
* core_state_t
*
* contains most recent values reported by the fly_controller. Should only be 
* written to by the flight controller after initialization.
*******************************************************************************/
typedef struct core_state_t{
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
} core_state_t;


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
* log_entry_t
*
* Struct definition to contain a single line of the log. For each log entry
* you wish to create. Fill in an instance of this and pass to add_log_entry()
*******************************************************************************/
#define CORE_LOG_TABLE \
	X(uint64_t,	"%ld", 	loop_index	) \
								  	  \
	X(float,  	"%f",	alt			) \
    X(float, 	"%f",	roll		) \
    X(float,  	"%f",	pitch		) \
    X(float,  	"%f",	yaw			) \
    							 	  \
    X(float,  	"%f",	u_thr		) \
	X(float,  	"%f",	u_roll		) \
    X(float,  	"%f",	u_pitch		) \
    X(float,  	"%f",	u_yaw		) \
	X(float,  	"%f",	u_X			) \
	X(float,  	"%f",	u_Y			) \
								 	  \
    X(float,  	"%f",	mot_1		) \
    X(float,  	"%f",	mot_2		) \
	X(float,  	"%f",	mot_3		) \
    X(float,  	"%f",	mot_4		) \
    X(float,  	"%f",	mot_5		) \
    X(float,  	"%f",	mot_6		) \
    X(float,  	"%f",	vbatt		)

#define X(type, fmt, name) type name ;
typedef struct log_entry_t { CORE_LOG_TABLE } log_entry_t;
#undef X


#endif // FLY_TYPES