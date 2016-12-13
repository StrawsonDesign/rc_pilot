/*******************************************************************************
* log_table.h
*
* values to be logged to 
*******************************************************************************/



#define LOG_TABLE \
	X(uint64_t,	"%ld", 	loop_index	) \
									  \
	X(float,	"%f",	alt			) \
	X(float,	"%f",	roll		) \
	X(float,	"%f",	pitch		) \
	X(float,	"%f",	yaw			) \
									  \
	X(float,	"%f",	u_thr		) \
	X(float,	"%f",	u_roll		) \
	X(float,	"%f",	u_pitch		) \
	X(float,	"%f",	u_yaw		) \
	X(float,	"%f",	u_X			) \
	X(float,	"%f",	u_Y			) \
									  \
	X(float,	"%f",	mot_1		) \
	X(float,	"%f",	mot_2		) \
	X(float,	"%f",	mot_3		) \
	X(float,	"%f",	mot_4		) \
	X(float,	"%f",	mot_5		) \
	X(float,	"%f",	mot_6		) \
	X(float,	"%f",	vbatt		)


/*******************************************************************************
* log_entry_t
*
* Struct definition to contain a single line of the log. For each log entry
* you wish to create. Fill in an instance of this and pass to add_log_entry()
*******************************************************************************/
#define X(type, fmt, name) type name ;
typedef struct log_entry_t { LOG_TABLE } log_entry_t;
#undef X

