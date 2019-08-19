/**
 * @file state_estimator.h
 *
 * @brief      Functions to start and stop the state estimator
 *
 * This runs at the same rate as the feedback controller.
 * state_estimator_march() is called immediately before  feedback_march() in the
 * IMU interrupt service routine.
 */

#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <rc/mpu.h>
#include <rc_pilot_defs.h>
#include <stdint.h>  // for uint64_t

/**
 * This is the output from the state estimator. It contains raw sensor values
 * and the outputs of filters. Everything is in NED coordinates defined as:
 *
 * - X pointing Rorward
 * - Y pointing Right
 * - Z pointing Down
 *
 * right hand rule applies for angular values such as tait bryan angles and gyro
 * - Positive Roll to the right about X
 * - Positive Pitch back about Y
 * - Positive Yaw right about Z
 */
typedef struct state_estimate_t
{
    int initialized;

    /** @name IMU (accel gyro)
     * Normalized Quaternion is straight from the DMP but converted to NED
     * coordinates. Tait-Bryan angles roll pitch and yaw angles are then
     * converted from the quaternion.
     * the roll_pitch_yaw values in the taid bryan angles tb_imu are bounded
     * by +-pi since they come straight from the quaternion. the state estimator
     * keeps track of these rotations and generates continuous_yaw which is
     * unbounded and keeps track of multiple rotations. This provides a continuously
     * differentiable variable with no jumps between +-pi
     */
    ///@{
    double gyro[3];             ///< gyro roll pitch yaw (rad/s)
    double accel[3];            ///< accel XYZ NED coordinates (m/s^2)
    double quat_imu[4];         ///< DMP normalized quaternion
    double tb_imu[3];           ///< tait bryan roll pitch yaw angle (rad)
    double imu_continuous_yaw;  ///< continuous yaw from imu only (multiple turns)
    ///@}

    /** @name IMU (magnetometer)
     * these values are only set when magnetometer is enabled in settings.
     * right now these aren't used and we don't suggest turning the magnetometer on
     */
    ///@{
    double mag[3];           ///< magnetometer XYZ NED coordinates ()
    double mag_heading_raw;  ///< raw compass heading
    double mag_heading;      ///< compass heading filtered with IMU
    double mag_heading_continuous;
    double quat_mag[4];  ///< quaterion filtered
    double tb_mag[3];    ///< roll pitch yaw with magetometer heading fixed (rad)
    ///@}

    /** @name selected values for feedback
     * these are copoies of other values in this state estimate used for feedback
     * this is done so we can easily chose which source to get feedback from (mag or no mag)
     */
    ///@{
    double roll;
    double pitch;
    double yaw;
    double continuous_yaw;  ///<  keeps increasing/decreasing aboce +-2pi
    double X;
    double Y;
    double Z;
    ///@}

    /** @name filtered data from IMU & barometer
     * Altitude estimates from kalman filter fusing IMU and BMP data.
     * Alttitude, velocity, and acceleration are in units of m, m/s, m/s^2
     * Note this is altitude so positive is upwards unlike the NED
     * coordinate frame that has Z pointing down.
     */
    ///@{
    double bmp_pressure_raw;  ///< raw barometer pressure in Pascals
    double alt_bmp_raw;       ///< altitude estimate using only bmp from sea level (m)
    double alt_bmp;           ///< altitude estimate using kalman filter (IMU & bmp)
    double alt_bmp_vel;       ///< z velocity estimate using kalman filter (IMU & bmp)
    double alt_bmp_accel;     ///< z accel estimate using kalman filter (IMU & bmp)
    ///@}

    /** @name Motion Capture data
     * As mocap drop in and out the mocap_running flag will turn on and off.
     * Old values will remain readable after mocap drops out.
     */
    ///@{
    int mocap_running;            ///< 1 if motion capture data is recent and valid
    uint64_t mocap_timestamp_ns;  ///< timestamp of last received packet in nanoseconds since boot
    double pos_mocap[3];          ///< position in mocap frame, converted to NED if necessary
    double quat_mocap[4];         ///< UAV orientation according to mocap
    double tb_mocap[3];           ///< Tait-Bryan angles according to mocap
    int is_active;                ///< TODO used by mavlink manager, purpose unclear... (pg)
    ///@}

    /** @name Global Position Estimate
     * This is the global estimated position, velocity, and acceleration
     * output of a kalman filter which filters accelerometer, DMP attitude,
     * and mocap data. If mocap is not available, barometer will be used.
     *
     * global values are in the mocap's frame for position control.
     * relative values are in a frame who's origin is at the position where
     * the feedback controller is armed. Without mocap data the filter will
     * assume altitude from the barometer and accelerometer, and position
     * estimate will have steady state gain of zero to prevent runaway.
     */
    ///@{
    double pos_global[3];
    double vel_global[3];
    double accel_global[3];
    double pos_relative[3];
    double vel_relative[3];
    double accel_relative[3];
    ///@}

    /** @name Other */
    ///@{
    double v_batt_raw;  ///< main battery pack voltage (v)
    double v_batt_lp;   ///< main battery pack voltage with low pass filter (v)
    double bmp_temp;    ///< temperature of barometer
                        ///@}

} state_estimate_t;

extern state_estimate_t state_estimate;
extern rc_mpu_data_t mpu_data;

/**
 * @brief      Initial setup of the state estimator
 *
 * barometer must be initialized first
 *
 * @return     0 on success, -1 on failure
 */
int state_estimator_init(void);

/**
 * @brief      March state estimator forward one step
 *
 * Called immediately before feedback_march
 *
 * @return     0 on success, -1 on failure
 */
int state_estimator_march(void);

/**
 * @brief      jobs the state estimator must do after feedback_controller
 *
 * Called immediately after feedback_march in the ISR. Currently this reads
 *
 * @return     0 on success, -1 on failure
 */
int state_estimator_jobs_after_feedback(void);

/**
 * @brief      Cleanup the state estimator, freeing memory
 *
 * @return     0 on success, -1 on failure
 */
int state_estimator_cleanup(void);

#endif  //  STATE_ESTIMATOR_H
