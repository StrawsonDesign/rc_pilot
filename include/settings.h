/**
 * <settings.h>
 *
 * @brief   Functions to read the json settings file
 *
 * @addtogroup Settings
 * @{
 */

#ifndef SETTINGS_H
#define SETTINGS_H

#include <rc/math/filter.h>
#include <rc/mpu.h>

#include <flight_mode.h>
#include <input_manager.h>
#include <mix.h>
#include <rc_pilot_defs.h>
#include <thrust_map.h>

/**
 * Configuration settings read from the json settings file and passed to most
 * threads as they initialize.
 */
typedef struct settings_t
{
    /** @name File details */
    char name[128];  ///< string declaring the name of the settings file
    ///@}

    /**@name warings */
    ///@{
    int warnings_en;
    ///@}

    /** @name physical parameters */
    ///@{
    int num_rotors;
    rotor_layout_t layout;
    int dof;
    thrust_map_t thrust_map;
    double v_nominal;
    int enable_magnetometer;  // we suggest leaving as 0 (mag OFF)
    ///@}

    /** @name flight modes */
    ///@{
    int num_dsm_modes;
    flight_mode_t flight_mode_1;
    flight_mode_t flight_mode_2;
    flight_mode_t flight_mode_3;
    ///@}

    /** @name dsm radio config */
    ///@{
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
    ///@}

    /** @name printf settings */
    ///@{
    int printf_arm;
    int printf_altitude;
    int printf_rpy;
    int printf_sticks;
    int printf_setpoint;
    int printf_u;
    int printf_xbee;
    int printf_motors;
    int printf_mode;
    ///@}

    /** @name log settings */
    ///@{
    int enable_logging;
    int log_sensors;
    int log_state;
    int log_setpoint;
    int log_control_u;
    int log_motor_signals;
    int log_throttles;
    int log_xbee;
    int log_dsm;
    int log_flight_mode;
    ///@}

    /** @name mavlink stuff */
    ///@{
    char dest_ip[24];
    uint8_t my_sys_id;
    uint16_t mav_port;

    /** @name feedback controllers */
    ///@{
    rc_filter_t roll_controller;
    rc_filter_t pitch_controller;
    rc_filter_t yaw_controller;
    rc_filter_t altitude_controller;
    rc_filter_t horiz_vel_ctrl_4dof;
    rc_filter_t horiz_vel_ctrl_6dof;
    rc_filter_t horiz_pos_ctrl_4dof;
    rc_filter_t horiz_pos_ctrl_6dof;
    double max_XY_velocity;
    double max_Z_velocity;
    ///@}

} settings_t;

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
int settings_print(void);

#endif  // SETTINGS_H

/* @} end group Settings */
