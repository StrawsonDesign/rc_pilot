/**
 * <setpoint_manager.h>
 *
 * @brief   Guidance module for the vehicle
 *
 * Setpoint manager runs at the same rate as the feedback controller
 * and is the interface between the user inputs (input manager) and
 * the feedback controller setpoint. currently it contains very
 * simply logic and runs very quickly which is why it's okay to run
 * in the feedback ISR right before the feedback controller. In the
 * future this is where go-home and other higher level autonomy will
 * live.
 *
 * This serves to allow the feedback controller to be as simple and
 * clean as possible by putting all high-level manipulation of the
 * setpoints here. Then feedback-controller only needs to march the
 * filters and zero them out when arming or enabling controllers
 *
 * @addtogroup SetpointManager
 * @{
 *
 */

#ifndef SETPOINT_MANAGER_H
#define SETPOINT_MANAGER_H

#include <rc_pilot_defs.h>

/**
 * Setpoint for the feedback controllers. This is written by setpoint_manager
 * and primarily read in by fly_controller. May also be read by printf_manager
 * and log_manager for telemetry
 */
typedef struct setpoint_t
{
    /** @name general */
    ///< @{
    int initialized;  ///< set to 1 once setpoint manager has initialized
    int en_6dof;      ///< enable 6DOF control features
    ///< @}

    /** @name direct passthrough
     * user inputs tranlate directly to mixing matrix
     */
    ///< @{
    double Z_throttle;      ///< used only when altitude controller disabled
    double X_throttle;      ///< only used when 6dof is enabled, positive forward
    double Y_throttle;      ///< only used when 6dof is enabled, positive right
    double roll_throttle;   ///< only used when roll_pitch_yaw controllers are disbaled
    double pitch_throttle;  ///< only used when roll_pitch_yaw controllers are disbaled
    double yaw_throttle;    ///< only used when roll_pitch_yaw controllers are disbaled
    ///< @}

    /** @name attitude setpoint */
    ///< @{
    int en_rpy_ctrl;  ///< enable the roll pitch yaw controllers
    double roll;      ///< roll angle (positive tip right) (rad)
    double pitch;     ///< pitch angle (positive tip back) (rad)
    double yaw;       ///< glabal yaw angle, positive left
    double yaw_dot;   ///< desired rate of change in yaw rad/s
    ///< @}

    /** @name altitude */
    ///< @{
    int en_Z_ctrl;  ///< enable altitude feedback.
    double Z;       ///< vertical distance from where controller was armed
    double Z_dot;   ///< vertical velocity m/s^2, remember Z points down
    ///< @}

    /** @name horizontal velocity setpoint */
    ///< @{
    int en_XY_vel_ctrl;
    double X_dot;
    double Y_dot;
    ///< @}

    /** @name horizontal velocity setpoint */
    ///< @{
    int en_XY_pos_ctrl;
    double X;
    double Y;
} setpoint_t;

extern setpoint_t setpoint;

/**
 * @brief      Initializes the setpoint manager.
 *
 * @return     0 on success, -1 on failure
 */
int setpoint_manager_init(void);

/**
 * @brief      updates the setpoint manager, call this before feedback loop
 *
 * @return     0 on success, -1 on failure
 */
int setpoint_manager_update(void);

/**
 * @brief      cleans up the setpoint manager, not really necessary but here for
 *             completeness
 *
 * @return     0 on clean exit, -1 if exit timed out
 */
int setpoint_manager_cleanup(void);

#endif  // SETPOINT_MANAGER_H

/* @} end group SetpointManager */
