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

#include <stdint.h> // for uint64_t
#include <rc_pilot_defs.h>

/**
 * This is the state of the feedback loop. contains most recent values
 * reported by the feedback controller. Should only be written to by the
 * feedback controller after initialization.
 */
typedef struct state_estimate_t{
	/** @name unfiltered basic sensor data */
	///@{
	double v_batt;		///< main battery pack voltage (v)
	double gyro[3];		///< gyro roll pitch yaw NED coordinates (rad/s)
	double accel[3];	///< accel XYZ NED coordinates (m/s^2)
	double altitude_bmp;	///< altitude estimate using only bmp from sea level (m)
	int mocap_running;	///< 1 if motion capture data is recent and valid
	double mocap_pos[3];	///< motion capture position in it's own frame, NED coordinates (m)
	double mocap_quat[4];	///< motion capture quaternion rotation of UAV
	double mocap_tb[3];	///< Tait-Bryan angles from mocap_quat for convenience.
	///@}

	/** @name filtered data */
	///@{
	double altitude_kf;	///< altitude estimate using kalman filter
	double alt_kf_vel;	///< z velocity estimate using kalman filter
	double alt_kf_accel;	///< z accel estimate using kalman filter
	double roll;		///< current roll angle (rad)
	double pitch;		///< current pitch angle (rad)
	double yaw;		///< current yaw angle (rad)
	///@}

} state_estimate_t;

extern state_estimate_t state_estimate;



/**
 * @brief      Initial setup of the state estimator
 *
 * barometer must be initialized first
 *
 * @return     0 on success, -1 on failure
 */
int state_estimator_init();


/**
 * @brief      March state estimator forward one step
 *
 * Called immediately before feedback_march
 *
 * @return     0 on success, -1 on failure
 */
int state_estimator_march();


/**
 * @brief      jobs the state estimator must do after feedback_controller
 *
 * Called immediately after feedback_march in the ISR. Currently this
 *
 * @return     0 on success, -1 on failure
 */
int state_estimator_jobs_after_feedback();


/**
 * @brief      Cleanup the state estimator, freeing memory
 *
 * @return     0 on success, -1 on failure
 */
int state_estimator_cleanup();




#endif //  STATE_ESTIMATOR_H

