/*******************************************************************************
* basic_settings.h
*
* These 
*******************************************************************************/

// Physical Configuration
#define ROTORS					6	 // 4, 6 or 8
#define ROTOR_LAYOUT			'X'	 // 'X' or '+'
#define DOF						4	 // 4 or 6
#define V_NOMINAL  				14.8 // 4S pack

// see robotics_cape.h for orientation options
#define BBB_ORIENTATION		ORIENTATION_Z_UP 

// 1 to enable logging, 0 to disable
#define ENABLE_LOGGING			1

// these are the two flight modes which be toggled between mid-flight via the
// DSM2 mode switch (default dsm2 channel 6). For available options see
// flight_mode_t in fly_types.h.
#define DSM2_MODE_1		ATTITUDE_DIRECT_THROTTLE
#define DSM2_MODE_2		ATTITUDE_ALTITUDE_HOLD