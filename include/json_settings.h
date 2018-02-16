/**
 * @file json_settings.h
 *
 * @brief      Functions to read the json settings file
 */

#ifndef JSON_SETTINGS_H
#define JSON_SETTINGS_H


#include <fly_types.h>


/**
 * @brief      Populates the setting sand controller structs with the json file.
 *
 *             Populates the settings and controller structs with the json file
 *             contents. If no settings file exits, it makes a new one filled
 *             with defaults. Used in json_settings.c
 *
 * @param      settings  The settings
 *
 * @return     0 on success, -1 on failure
 */
int load_settings_from_file(fly_settings_t* settings);

/**
 * @brief      Only used in debug mode. (prints settings to console?)
 *
 *             Used in json_settings.c
 *
 * @return     0 on success, -1 on failure
 */
int print_settings();

/**
 * @brief      gets the (roll?) controllers read from the last json read.  !!!
 *
 *             Used in json_settings.c
 *
 * @param      ctrl  The control
 *
 * @return     0 on success, -1 on failure
 */
int get_json_roll_controller(rc_filter_t* ctrl);

/**
 * @brief      gets the (pitch?) controllers read from the last json read.  !!!
 *
 *             Used in json_settings.c
 *
 * @param      ctrl  The control
 *
 * @return     0 on success, -1 on failure
 */
int get_json_pitch_controller(rc_filter_t* ctrl);

/**
 * @brief      gets the (yaw?) controllers read from the last json read.  !!!
 *
 *             Used in json_settings.c
 *
 * @param      ctrl  The control
 *
 * @return     0 on success, -1 on failure
 */
int get_json_yaw_controller(rc_filter_t* ctrl);

/**
 * @brief      gets the (altitude?) controllers read from the last json read.
 *             !!!
 *
 *             Used in json_settings.c
 *
 * @param      ctrl  The control
 *
 * @return     0 on success, -1 on failure
 */
int get_json_altitude_controller(rc_filter_t* ctrl);

#endif // JSON_SETTINGS_H