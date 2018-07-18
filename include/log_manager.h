/**
 * @file log_manager.h
 *
 * @brief      Functions to start, stop, and interact with the log manager
 *             thread.
 */

#ifndef LOG_MANAGER_H
#define LOG_MANAGER_H


// to allow printf macros for multi-architecture portability
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

/**
 * table of values to go into each log entry. It is structured this way so that
 * it can be transformed into a struct or fprintf statement with macros.
 */
#define LOG_TABLE \
	X(uint64_t,	"%" PRIu64,	loop_index	) \
	X(uint64_t,	"%" PRIu64,	last_step_ns	) \
							  \
	X(double,	"%f",		altitude_kf	) \
	X(double,	"%f",		altitude_bmp)	  \
	X(double,	"%f",		roll		) \
	X(double,	"%f",		pitch		) \
	X(double,	"%f",		yaw		) \
							  \
	X(double,	"%f",		Z_throttle_sp	) \
	X(double,	"%f",		altitude_sp	) \
	X(double,	"%f",		roll_sp		) \
	X(double,	"%f",		pitch_sp	) \
	X(double,	"%f",		yaw_sp		) \
							\
	X(double,	"%f",		u_X		) \
	X(double,	"%f",		u_Y		) \
	X(double,	"%f",		u_Z		) \
	X(double,	"%f",		u_roll		) \
	X(double,	"%f",		u_pitch		) \
	X(double,	"%f",		u_yaw		) \
							  \
	X(double,	"%f",		mot_1		) \
	X(double,	"%f",		mot_2		) \
	X(double,	"%f",		mot_3		) \
	X(double,	"%f",		mot_4		) \
	X(double,	"%f",		mot_5		) \
	X(double,	"%f",		mot_6		) \
	X(double,	"%f",		v_batt		)


#define X(type, fmt, name) type name ;
/**
 * Struct definition to contain a single line of the log. For each log entry you
 * wish to create. Fill in an instance of this and pass to add_log_entry()
 */
typedef struct log_entry_t { LOG_TABLE } log_entry_t;
#undef X


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
