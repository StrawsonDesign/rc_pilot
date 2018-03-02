/**
 * @file other.h
 *
 * @brief      Functions that don't fit anywhere else
 */

/**
 * @brief      Applies a dead zone to an input stick to prevent drift.
 *
 * @param[in]  in    joystick position (-1 to 1)
 * @param[in]  zone  distance from 0 the deadzone extends in + and - directions
 *
 * @return     0 on success, -1 on failure
 */
float apply_deadzone(float in, float zone);

/**
 * @brief      sends signal 0 to all motor channels
 *
 *             Used in other.c
 *
 * @param[in]  rotors  The rotor id
 * @param[in]  val     pulse signal strength, should be 0???
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
