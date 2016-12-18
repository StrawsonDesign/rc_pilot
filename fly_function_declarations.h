/*******************************************************************************
* fly_function_declarations.h
*
* Declarations for high level functions intended to be called across C files.
* This does not include functions who's use remains contained in one C file.
*******************************************************************************/
#include "fly_types.h"

////////////////////////////////////////////////////////////////////////////////
// battery_manager.c
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
* int start_battery_manager(cstate_t* core_state, fly_settings_t* fly_settings)
*
* 
*******************************************************************************/
int start_battery_manager(cstate_t* core_state, fly_settings_t* fly_settings);

/*******************************************************************************
* int join_battery_manager_thread()
*
* Waits for the battery manager thread to exit. Returns 0 if the thread exited 
* cleanly. Returns -1 if the exit timed out.
* This should only be called after the program flow state is set to EXITING as 
* that's the only thing that will cause the thread to exit on its own safely.
*******************************************************************************/
int join_battery_manager_thread();


////////////////////////////////////////////////////////////////////////////////
// feedback_controller.c
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
* int disarm_controller()
*	
* This is how outside functions should stop the flight controller.
*******************************************************************************/
int disarm_controller();

/*******************************************************************************
* int disarm_controller()
*	
* This is how outside functions should stop the flight controller.
*******************************************************************************/
int arm_controller();

/*******************************************************************************
* arm_state_t get_controller_arm_state()
*	
* Returns the arm state of the controller so outside functions, namely the
* setpoint_manager, can tell if the controller is armed or not.
*******************************************************************************/
arm_state_t get_controller_arm_state();

/*******************************************************************************
* initialize_controller()
*
* initial setup of all feedback controllers. Should only be called once on
* program start. 
*******************************************************************************/
int initialize_controller(cstate_t* cstate, setpoint_t* setpoint, \
									imu_data_t* imu_data, user_input_t* ui);

/*******************************************************************************
* fly_controller()
*	
* Should be called by the IMU interrupt at SAMPLE_RATE_HZ
*******************************************************************************/
int fly_controller();





////////////////////////////////////////////////////////////////////////////////
// input_manager.c
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
* int start_input_manager(user_input_t* user_input, fly_settings_t* settings)
*
* Watch for new DSM data and translate into local user mode
*******************************************************************************/
int start_input_manager(user_input_t* user_input, fly_settings_t* settings);

/*******************************************************************************
* int join_input_manager_thread()
*
* Waits for the input manager thread to exit. Returns 0 if the input manager
* thread exited cleanly. Returns -1 if the exit timed out and the thread had
* to be force closed.
* This should only be called after the program flow state is set to EXITING as 
* that's the only thing that will cause the thread to exit on its own safely.
*******************************************************************************/
int join_input_manager_thread();





////////////////////////////////////////////////////////////////////////////////
// json_settings.c
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
* int load_settings_from_file(fly_settings_t* settings)
*
* populates the settings and controller structs with the json file contents
* if no settings file exists, it makes a new one filled with defaults
*******************************************************************************/
int load_settings_from_file(fly_settings_t* settings);

/*******************************************************************************
* int print_settings(json_object* jobj)
*
* only used in debug mode
*******************************************************************************/
int print_settings();

/*******************************************************************************
* int get_json_roll_controller(d_filter_t* ctrl)
*
* gets the controllers read from the last json read.
* returns 0 on success or -1 on failure
*******************************************************************************/
int get_json_roll_controller(d_filter_t* ctrl);

/*******************************************************************************
* int get_json_pitch_controller(d_filter_t* ctrl)
*
* gets the controllers read from the last json read.
* returns 0 on success or -1 on failure
*******************************************************************************/
int get_json_pitch_controller(d_filter_t* ctrl);

/*******************************************************************************
* int get_json_yaw_controller(d_filter_t* ctrl)
*
* gets the controllers read from the last json read.
* returns 0 on success or -1 on failure
*******************************************************************************/
int get_json_yaw_controller(d_filter_t* ctrl);

/*******************************************************************************
* int get_json_altitude_controller(d_filter_t* ctrl)
*
* gets the controllers read from the last json read.
* returns 0 on success or -1 on failure
*******************************************************************************/
int get_json_altitude_controller(d_filter_t* ctrl);





////////////////////////////////////////////////////////////////////////////////
// setpoint_manager.c
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
* int start_setpoint_manager(setpoint_t* setpoint, user_input_t* user_input, \
							cstate_t* cstate, fly_settings_t* settings)
*
* Starts the setpoint manager thread
*******************************************************************************/
int start_setpoint_manager(setpoint_t* setpoint, user_input_t* user_input, \
							cstate_t* cstate, fly_settings_t* settings);

/*******************************************************************************
* int join_setpoint_manager_thread()
*
* Waits for the setpoint manager thread to exit. Returns 0 if the thread exited 
* cleanly. Returns -1 if the exit timed out.
*******************************************************************************/
int join_setpoint_manager_thread();



////////////////////////////////////////////////////////////////////////////////
// log_manager.c
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
* int start_log_manager()
*
* Create a new csv log file and start the background thread
*******************************************************************************/
int start_log_manager();

/*******************************************************************************
* int print_entry(log_entry_t entry)
*
* write the contents of one entry to the console
*******************************************************************************/
int print_entry(log_entry_t entry);

/*******************************************************************************
* int add_log_entry(log_entry_t new_entry)
*
* quickly add new data to local buffer
*******************************************************************************/
int add_log_entry(log_entry_t new_entry);

/*******************************************************************************
* stop_log_manager()
*
* finish writing remaining data to log and close it.
* return -1 if there was a timeout and the thread had to force close.
*******************************************************************************/
int stop_log_manager();




////////////////////////////////////////////////////////////////////////////////
// mixing_matrix.c
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
* int initialize_mixing_matrix(layout_t layout)
*
* For a given number of rotors, layout character ('X','+') and degrees of
* freedom in control input (4 or 6), this selects the correct predefined
* mixing matrix from mixing_matrix_defs.h. The matrix is kept locally in
* mixing_matrix.c to prevent accidental misuse or modification. Use the other
* functions here to interface with it.
*******************************************************************************/
int initialize_mixing_matrix(layout_t layout);

/************************************************************************
* int mix_all_controls(double u[6], double* mot)
*	
* fills the vector mot with the linear combination of roll pitch
* yaw and throttle based on mixing matrix. if dof=6, X and Y are also
* added. Outputs are blindly saturated between 0 and 1. This is for rudimentary 
* mixing and testing only. It is recommended to check for saturation for each 
* input with check_channel_saturation then add inputs sequentially with 
* add_mixed_input() instead.
*******************************************************************************/
int mix_all_controls(double u[6], double* mot);

/*******************************************************************************
* int check_channel_saturation(int ch, double* mot, double* min, double* max)
*	
* checks to see if the desired input u on channel ch corresponding to throttle,
* roll, pitch, etc would saturate a motor in the given motor array mot. If 
* saturation would occur, it returns the maximum allowed input u.
*******************************************************************************/
int check_channel_saturation(int ch, double* mot, double* min, double* max);

/*******************************************************************************
* int add_mixed_input(double u, int ch, double* mot)
*	
* Mixes the control input u for a single channel ch corresponding to throttle, 
* roll pitch etc to the existing motor array mot. No saturation is done, the
* input u should be checked for saturation validity with 
* check_channel_saturation() first.
*******************************************************************************/
int add_mixed_input(double u, int ch, double* mot);



////////////////////////////////////////////////////////////////////////////////
// other.c
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
* double apply_dead_zone(double in, double zone)
*
* Applies a dead zone to an input stick in. in is supposed to range from -1 to 1
* the dead zone is centered around 0. zone specifies the distance from 0 the
* zone extends.
*******************************************************************************/
double apply_deadzone(double in, double zone);	

/*******************************************************************************
* int send_pulse_to_rotors(int rotors, double val)
*
* sends signal 0 to all motor channels
*******************************************************************************/
int send_pulse_to_rotors(int rotors, double val);

/*******************************************************************************
* int pause_pressed_func()
*
* Disarm controller on momentary press.
* If the user holds the pause button for BUTTON_EXIT_TIME_S, exit cleanly.
*******************************************************************************/
int pause_pressed_func();

/*******************************************************************************
* int on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
int on_pause_released();






////////////////////////////////////////////////////////////////////////////////
// printf_manager.c
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
* int start_printf_manager(cstate_t* cstate,	setpoint_t* setpoint, \
					user_input_t* user_input, fly_settings_t* settings)
*
* Start the printf_manager which should be the only thing printing to the screen
* besides error messages from other threads.
*******************************************************************************/
int start_printf_manager(cstate_t* cstate,	setpoint_t* setpoint, \
					user_input_t* user_input, fly_settings_t* settings);

/*******************************************************************************
* int join_printf_manager_thread()
*
* Waits for the printf_manager thread to exit. Returns 0 if the thread exited 
* cleanly. Returns -1 if the exit timed out.
*******************************************************************************/
int join_printf_manager_thread();




////////////////////////////////////////////////////////////////////////////////
// thrust_map.c
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
* int initialize_thrust_map(thrust_map_t map)
*
* check the thrust map for validity and populate data arrays
*******************************************************************************/
int initialize_thrust_map(thrust_map_t map);

/*******************************************************************************
* double map_motor_signal(double m)
*
* Corrects the motor signal m for non-linear thrust curve in palce. Returns -1
* on error, otherwise 0
*******************************************************************************/
double map_motor_signal(double m);










