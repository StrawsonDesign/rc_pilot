/**
 * <fly/settings.h>
 *
 * @brief      Functions to read the json settings file
 */

#ifndef SETTINGS_H
#define SETTINGS_H

#include <rc/math/filter.h>
#include <rc/mpu.h>

#include <flight_mode.h>
#include <thrust_map.h>
#include <mix.h>
#include <input_manager.h>
#include <rc_pilot_defs.h>



/**
 * Configuration settings read from the json settings file and passed to most
 * threads as they initialize.
 */
typedef struct settings_t{
	char name[128]; ///< string declaring the name of the settings file

	// physical parameters
	int num_rotors;
	rotor_layout_t layout;
	int dof;
	thrust_map_t thrust_map;
	double v_nominal;
	int feedback_hz;

	// features
	int enable_logging;
	int enable_magnetometer;

	// flight modes
	int num_dsm_modes;
	flight_mode_t flight_mode_1;
	flight_mode_t flight_mode_2;
	flight_mode_t flight_mode_3;


	// dsm radio config
	int dsm_thr_ch;
	int dsm_thr_pol;
	int dsm_roll_ch;
	int dsm_roll_pol;
	int dsm_pitch_ch;
	int dsm_pitch_pol;
	int dsm_yaw_ch;
	int dsm_yaw_pol;
	int dsm_mode_ch;
	int dsm_mode_pol;
	dsm_kill_mode_t dsm_kill_mode;
	int dsm_kill_ch;
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

	// mavlink stuff
	char dest_ip[24];
	uint8_t my_sys_id;
	uint16_t mav_port;

}settings_t;

/**
 * settings are external, so just include this header and read from it
 */
extern settings_t settings;

/**
 * @brief      Populates the settings and controller structs with the settings file.
 *
 * @return     0 on success, -1 on failure
 */
int settings_load_from_file(char* path);


/**
 * @brief      Only used in debug mode. Prints settings to console
 *
 * @return     0 on success, -1 on failure
 */
int settings_print();


/**
 * @brief      gets the roll controllers read from the last json read.
 *
 * @param      ctrl  pointer to the roll controllers to retrieve
 *
 * @return     0 on success, -1 on failure
 */
int settings_get_roll_controller(rc_filter_t* ctrl);


/**
 * @brief      gets the pitch controllers read from the last json read.
 *
 * @param      ctrl  pointer to the pitch controllers to retrieve
 *
 * @return     0 on success, -1 on failure
 */
int settings_get_pitch_controller(rc_filter_t* ctrl);


/**
 * @brief      gets the yaw controllers read from the last json read.
 *
 * @param      ctrl  pointer to the yaw controllers to retrieve
 *
 * @return     0 on success, -1 on failure
 */
int settings_get_yaw_controller(rc_filter_t* ctrl);


/**
 * @brief      gets the altitude controllers read from the last json read.
 *
 * @param      ctrl  pointer to the altitude controllers to retrieve
 *
 * @return     0 on success, -1 on failure
 */
int settings_get_altitude_controller(rc_filter_t* ctrl);

#endif // SETTINGS_H
