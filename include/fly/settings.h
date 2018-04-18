/**
 * <settings.h>
 *
 * @brief      Functions to read the json settings file
 */

#ifndef SETTINGS_H
#define SETTINGS_H


#include <rc/math/filter.h>

/**
 * @brief      determines how the setpoint manager behaves
 *
 *             DSM_KILL_DEDICATED_SWITCH: A dedicated channel is used as a kill
 *             switch. Carefully set the dsm_kill_ch and dsm_kill_pol channel
 *             and olarity settings.
 *
 *             DSM_KILL_NEGATIVE_THROTTLE: Some radios, such as Spektrum DXe
 *             have an ARM/DISARM switch which forces the throttle channel down
 *             below normal range to disarm. This frees up a channel for other
 *             use and is the preffered method. When using this mode,
 *             dsm_kill_ch and dsm_kill_pol are ignored.
 */
typedef enum dsm_kill_mode_t{
	DSM_KILL_DEDICATED_SWITCH,
	DSM_KILL_NEGATIVE_THROTTLE
} dsm_kill_mode_t;


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

} fly_settings_t;



/**
 * @brief      Populates the setting sand controller structs with the json file.
 *
 *             If no settings file exits, it makes a new one filled
 *             with defaults. Used in json_settings.c
 *
 * @param      settings  pointer to flight setings struct
 *
 * @return     0 on success, -1 on failure
 */
int load_settings_from_file(fly_settings_t* settings);

/**
 * @brief      Only used in debug mode. Prints settings to console
 *
 * @return     0 on success, -1 on failure
 */
int print_settings();

/**
 * @brief      gets the roll controllers read from the last json read.
 *
 * @param      ctrl  pointer to the roll controllers to retrieve
 *
 * @return     0 on success, -1 on failure
 */
int get_json_roll_controller(rc_filter_t* ctrl);

/**
 * @brief      gets the pitch controllers read from the last json read.
 *
 * @param      ctrl  pointer to the pitch controllers to retrieve
 *
 * @return     0 on success, -1 on failure
 */
int get_json_pitch_controller(rc_filter_t* ctrl);

/**
 * @brief      gets the yaw controllers read from the last json read.
 *
 * @param      ctrl  pointer to the yaw controllers to retrieve
 *
 * @return     0 on success, -1 on failure
 */
int get_json_yaw_controller(rc_filter_t* ctrl);

/**
 * @brief      gets the altitude controllers read from the last json read.
 *
 * @param      ctrl  pointer to the altitude controllers to retrieve
 *
 * @return     0 on success, -1 on failure
 */
int get_json_altitude_controller(rc_filter_t* ctrl);

#endif // JSON_SETTINGS_H
