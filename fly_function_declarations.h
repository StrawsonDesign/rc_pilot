/**
 * @file fly_function_declarations.h
 *
 * @brief Declarations for high level functions intended to be called across C files.
 * This does not include functions who's use remains contained in one C file.
 *
 * [detailed description]
 *
 * @author
 * @date
 */

#include "fly_types.h"

////////////////////////////////////////////////////////////////////////////////
// battery_manager.c
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      Starts a battery manager thread.
 *
 *             Used in battery_manager.c
 *
 * @param      core_state    The core state !!!
 * @param      fly_settings  The fly settings !!!
 *
 * @return     0 on success, -1 on failure
 */
int start_battery_manager(cstate_t* core_state, fly_settings_t* fly_settings);

/**
 * @brief      Waits for the battery manager thread to exit.
 *
 *             This should only be called after the program flow state is set to
 *             EXITING as that's the only thing that will cause the thread to
 *             exit on its own safely. Used in battery_manager.c
 *
 * @return     0 if thread exited cleanly, -1 if exit timed out.
 */
int join_battery_manager_thread();


////////////////////////////////////////////////////////////////////////////////
// feedback_controller.c
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      This is how outside functions should stop the flight controller.
 *
 *             Used in feedback_controller.c
 *
 * @return     0 on success, -1 on failure
 */
int disarm_controller();

/**
 * @brief      This is how outside functions should start the flight controller.
 *
 *             Used in feedback_controller.c
 *
 * @return     0 on success, -1 on failure
 */
int arm_controller();

/**
 * @brief      Gets the controller arm state.
 *
 *             Returns the arm state of the controller so outside functions,
 *             namely the setpoint_manager, can tell if the controller is armed
 *             or not. Used in feedback_controller.c
 *
 * @return     The controller arm state.
 */
arm_state_t get_controller_arm_state();

/**
 * @brief      Initial setup of all feedback controllers. Should only be called
 *             once on program start.
 *
 *             Used in feedback_controller.c
 *
 * @param      cstate    The cstate
 * @param      setpoint  The setpoint
 * @param      imu_data  The imu data
 * @param      settings  The settings
 *
 * @return     0 on success, -1 on failure
 */
int initialize_controller(cstate_t* cstate, setpoint_t* setpoint, \
							rc_imu_data_t* imu_data, fly_settings_t* settings);

/**
 * @brief      Should be called by the IMU inturrupt at SAMPLE_RATE_HZ.
 *
 *             Used in feedback_controller.c
 *
 * @return     0 on success, -1 on failure
 */
int fly_controller();


////////////////////////////////////////////////////////////////////////////////
// input_manager.c
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      Starts an input manager thread.
 *
 *             Watch for new DSM data and translate into local user mode. Used
 *             in input_manager.c
 *
 * @param      user_input  The user input
 * @param      settings    The settings
 *
 * @return     0 on success, -1 on failure
 */
int start_input_manager(user_input_t* user_input, fly_settings_t* settings);

/**
 * @brief      Waits for the input manager thread to exit
 *
 *             This should only be called after the program flow state is set to
 *             EXITING as that's the only thing that will cause the thread to
 *             exit on its own safely. Used in input_manager.c
 *
 * @return     0 on clean exit, -1 if exit timed out.
 */
int join_input_manager_thread();


////////////////////////////////////////////////////////////////////////////////
// json_settings.c
////////////////////////////////////////////////////////////////////////////////

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


////////////////////////////////////////////////////////////////////////////////
// setpoint_manager.c
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      Starts the setpoint manager thread.
 *
 *             Used in setpoint_manager.c
 *
 * @param      setpoint    The setpoint
 * @param      user_input  The user input
 * @param      cstate      The cstate
 * @param      settings    The settings
 *
 * @return     0 on success, -1 on failure
 */
int start_setpoint_manager(setpoint_t* setpoint, user_input_t* user_input, \
							cstate_t* cstate, fly_settings_t* settings);

/**
 * @brief      Waits for the setpoint manager thread to exit.
 *
 *             Used in setpoint_manager.c
 *
 * @return     0 on clean exit, -1 if exit timed out
 */
int join_setpoint_manager_thread();


////////////////////////////////////////////////////////////////////////////////
// log_manager.c
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      creates a new csv log file and starts the background thread.
 *
 *             Used in log_manager.c
 *
 * @return     0 on success, -1 on failure
 */
int start_log_manager();

/**
 * @brief      Write the contents of one entry to the console.
 *
 *             Used in log_manager.c
 *
 * @param[in]  entry  The entry
 *
 * @return     0 on success, -1 on failure
 */
int print_entry(log_entry_t entry);

/**
 * @brief      quickly add new data to local buffer
 *
 *             Used in log_manager.c
 *
 * @param[in]  new_entry  The new entry
 *
 * @return     0 on success, -1 on failure
 */
int add_log_entry(log_entry_t new_entry);

/**
 * @brief      Finish writing remaining data to log and close thread.
 *
 *             Used in log_manager.c
 *
 * @return     0 on sucess and clean exit, -1 on exit timeout/force close.
 */
int stop_log_manager();


////////////////////////////////////////////////////////////////////////////////
// mixing_matrix.c
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      Initiallizes the mixing matrix for a given input layout.
 *
 *             For a given number of rotors, layout character ('X','+') and
 *             degrees of freedom in control input (4 or 6), this selects the
 *             correct predefined mixing matrix from mixing_matrix_defs.h. The
 *             matrix is kept locally in mixing _matrix.c to prevent accidental
 *             misuse or modification. Use the other function from
 *             mixing_matrix.c below to interface with it. Used in
 *             mixing_matrix.c
 *
 * @param[in]  layout  The layout
 *
 * @return     0 on success, -1 on failure
 */
int initialize_mixing_matrix(layout_t layout);

/**
 * @brief      Fills the vector mot with the linear combination of roll pitch
 *             yaw and throttle based on mixing matrix.
 *
 *             Fills the vector mot with the linear combination of roll pitch
 *             yaw and throttle based on mixing matrix. If dof = 6, X and Y are
 *             also added. Outputs are blindly saturated between 0 and 1. This
 *             is for rudimentary mixing and testing only. It is recommended to
 *             check for saturation for each input with check_channel_saturation
 *             then add inputs sequentially with add_mixed_input() instead. Used
 *             in mixing_matrix.c
 *
 * @param      u     { parameter_description }
 * @param      mot   The mot
 *
 * @return     0 on success, -1 on failure
 */
int mix_all_controls(float u[6], float* mot);

/**
 * @brief      Checks to see if the desired input u on channel ch would saturate
 *             a motor.
 *
 *             Checks to see if the desired input u on channel ch corresponding
 *             to throttle, roll, pitch, etc would saturate a motor in the given
 *             motor array mot. If saturation would occur, it returns the
 *             maximum allowed input u. Used in mixing_matrix.c
 *
 * @param[in]  ch    { parameter_description }
 * @param      mot   The mot
 * @param      min   The minimum
 * @param      max   The maximum
 *
 * @return     maximum allowed input u if saturation occurs. -1 if saturation
 *             does not occur. !!!
 */
int check_channel_saturation(int ch, float* mot, float* min, float* max);

/**
 * @brief      Mixes the control input u for a single channel ch to the existing
 *             motor array mot.
 *
 *             Mixes the control input u for a single channel ch corresponding
 *             to throttle roll pitch etc to the existing motor array mot. No
 *             saturation is done, the input u should be checked for saturation
 *             validity with check_channel_saturation() first. Used in
 *             mixing_matrix.c
 *
 * @param[in]  u     { parameter_description }
 * @param[in]  ch    { parameter_description }
 * @param      mot   The mot
 *
 * @return     0 on success, -1 on failure
 */
int add_mixed_input(float u, int ch, float* mot);


////////////////////////////////////////////////////////////////////////////////
// other.c
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      Applies a dead zone to an input stick in.
 *
 *             In is supposed to range from -1 to 1. The dead zone is centered
 *             around 0. zone specifies the distance from 0 the zone extends.
 *             Used in other.c
 *
 * @param[in]  in    { parameter_description }
 * @param[in]  zone  The zone
 *
 * @return     0 on success, -1 on failure
 */
float apply_deadzone(float in, float zone);

/**
 * @brief      sends signal 0 to all motor channels
 *
 *             Used in other.c
 *
 * @param[in]  rotors  The rotors
 * @param[in]  val     The value
 *
 * @return     0 on success, -1 on failure
 */
int send_pulse_to_rotors(int rotors, float val);

/**
 * @brief      Disarm the controller on momentary press. If the user holds the
 *             pause button for BUTTON_EXIT_TIME_S, exit cleanly.
 *
 *             Used in other.c
 */
void pause_pressed_func();

/**
 * @brief      Make the Pause button toggle between paused and running states.
 *
 *             Used in other.c
 */
void on_pause_released();


////////////////////////////////////////////////////////////////////////////////
// printf_manager.c
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      Start the printf_manager (thread?) which should be the only thing
 *             printing to the screen besides error messages from other threads.
 *             !!!
 *
 *             Used in printf_manager.c
 *
 * @param      cstate      The cstate
 * @param      setpoint    The setpoint
 * @param      user_input  The user input
 * @param      settings    The settings
 *
 * @return     0 on success, -1 on failure
 */
int start_printf_manager(cstate_t* cstate, setpoint_t* setpoint, \
			user_input_t* user_input, fly_settings_t* settings);

/**
 * @brief      Waits for the printf manager thread to exit.
 *
 *             Used in printf_manager.c
 *
 * @return     0 on clean exit, -1 on exit time out/force close
 */
int join_printf_manager_thread();


////////////////////////////////////////////////////////////////////////////////
// thrust_map.c
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      Check the thrust map for validity and populate data arrays.
 *
 *             Used in thrust_map.c
 *
 * @param[in]  map   The map
 *
 * @return     0 on success, -1 on failure
 */
int initialize_thrust_map(thrust_map_t map);

/**
 * @brief      Corrects the motor signal m for non-linear thrust curve in place.
 *
 *             Used in thrust_map.c
 *
 * @param[in]  m     { parameter_description }
 *
 * @return     0 on success, -1 on error
 */
float map_motor_signal(float m);










