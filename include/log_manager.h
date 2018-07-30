/**
 * <log_manager.h>
 *
 * @brief      Functions to start, stop, and interact with the log manager
 *             thread.
 *
 */

#ifndef LOG_MANAGER_H
#define LOG_MANAGER_H


/**
 * Struct containing all possible values that could be writen to the log. For
 * each log entry you wish to create, fill in an instance of this and pass to
 * add_log_entry(). You do not need to populate all parts of the struct.
 * Currently feedback.c populates all values and log_manager.c only writes the
 * values enabled in the settings file.
 */
typedef struct log_entry_t{
	/** @name index, always printed */
	///@{
	uint64_t loop_index, // timing
	uint64_t last_step_ns,
	///@}

	/** @name sensors */
	///@{
	double	v_batt
	double	altitude_bmp,
	double	gyro_roll,
	double	gyro_pitch,
	double	gyro_yaw,
	double	accel_X,
	double	accel_Y,
	double	accel_Z,
	///@}

	/** @name state estimate */
	///@{
	double	altitude_kf,
	double	roll,
	double	pitch,
	double	yaw,
	double	pos_X,
	double	pos_Y,
	double	pos_Z,
	///@}

	/** @name setpoint */
	///@{
	double	altitude_sp,
	double	roll_sp,
	double	pitch_sp,
	double	yaw_sp,
	///@}

	/** @name orthogonal control outputs */
	///@{
	double	u_X,
	double	u_Y,
	double	u_Z,
	double	u_roll,
	double	u_pitch,
	double	u_yaw,
	///@}

	/** @name motor signals */
	///@{
	double	mot_1,
	double	mot_2,
	double	mot_3,
	double	mot_4,
	double	mot_5,
	double	mot_6,
	double	mot_7
	double	mot_8
	///@}

} log_entry_t;



/**
 * @brief      creates a new csv log file and starts the background thread.
 *
 *             Used in log_manager.c
 *
 * @return     0 on success, -1 on failure
 */
int log_manager_init();

/**
 * @brief      Write the contents of one entry to the console.
 *
 *             Used in log_manager.c
 *
 * @param[in]  entry  The log_entry_t holding the LOG_TABLE to be printed.
 *
 * @return     0 on success, -1 on failure
 */
int print_entry(log_entry_t entry);

/**
 * @brief      quickly add new data to local buffer
 *
 *             Used in log_manager.c
 *
 * @param[in]  new_entry  the log_entry_t to be written to the buffer
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
int log_manager_cleanup();

#endif // LOG_MANAGER_H
