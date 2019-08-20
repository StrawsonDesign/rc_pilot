/**
 * <log_manager.h>
 *
 * @brief   Functions to start, stop, and interact with the log manager
 * thread.
 *
 * @addtogroup LogManager
 * @{
 */

#ifndef LOG_MANAGER_H
#define LOG_MANAGER_H

/**
 * Struct containing all possible values that could be writen to the log. For
 * each log entry you wish to create, fill in an instance of this and pass to
 * add_log_entry(void). You do not need to populate all parts of the struct.
 * Currently feedback.c populates all values and log_manager.c only writes the
 * values enabled in the settings file.
 */
typedef struct log_entry_t
{
    /** @name index, always printed */
    ///@{
    uint64_t loop_index;
    uint64_t last_step_ns;
    uint64_t imu_time_ns;  ///< Time that ISR occurs
    ///@}

    /** @name sensors */
    ///@{
    double v_batt;
    double alt_bmp_raw;
    double gyro_roll;
    double gyro_pitch;
    double gyro_yaw;
    double accel_X;
    double accel_Y;
    double accel_Z;
    ///@}

    /** @name state estimate */
    ///@{
    double roll;
    double pitch;
    double yaw;
    double X;
    double Y;
    double Z;
    double Xdot;
    double Ydot;
    double Zdot;
    ///@}

    /*** @name xbee data */
    ///@{
    uint32_t xbee_time;
    uint64_t xbee_time_received_ns;
    float xbee_x;
    float xbee_y;
    float xbee_z;
    float xbee_qw;
    float xbee_qx;
    float xbee_qy;
    float xbee_qz;
    ///@}

    /** @name throttles */
    ///@{
    double X_throttle;
    double Y_throttle;
    double Z_throttle;
    double roll_throttle;
    double pitch_throttle;
    double yaw_throttle;
    ///@}

    /** @name setpoint */
    ///@{
    double sp_roll;
    double sp_pitch;
    double sp_yaw;
    double sp_X;
    double sp_Y;
    double sp_Z;
    double sp_Xdot;
    double sp_Ydot;
    double sp_Zdot;
    ///@}

    /** @name orthogonal control outputs */
    ///@{
    double u_roll;
    double u_pitch;
    double u_yaw;
    double u_X;
    double u_Y;
    double u_Z;
    ///@}

    /** @name motor signals */
    ///@{
    double mot_1;
    double mot_2;
    double mot_3;
    double mot_4;
    double mot_5;
    double mot_6;
    double mot_7;
    double mot_8;
    ///@}

    /** @name dsm connection valid */
    ///@{
    int dsm_con;
    ///@}

    /** @name flight mode */
    ///@{
    int flight_mode;
    ///@}
} log_entry_t;

/**
 * @brief   creates a new csv log file and starts the background thread.
 *
 * @return  0 on success, -1 on failure
 */
int log_manager_init(void);

/**
 * @brief   quickly add new data to local buffer
 *
 * This is called after feedback_march after signals have been sent to
 * the motors.
 *
 * @return  0 on success, -1 on failure
 */
int log_manager_add_new();

/**
 * @brief   Finish writing remaining data to log and close thread.
 *
 * Used in log_manager.c
 *
 * @return  0 on sucess and clean exit, -1 on exit timeout/force close.
 */
int log_manager_cleanup(void);

#endif  // LOG_MANAGER_H

/* @} end group LogManager */
