/*******************************************************************************
* log_table.h
*
* values to be logged to 
*******************************************************************************/



#define LOG_TABLE \
	X(uint64_t,	"%lld",	loop_index	) \
									  \
	X(double,	"%f",	alt			) \
	X(double,	"%f",	roll		) \
	X(double,	"%f",	pitch		) \
	X(double,	"%f",	yaw			) \
									  \
	X(double,	"%f",	u_thr		) \
	X(double,	"%f",	u_roll		) \
	X(double,	"%f",	u_pitch		) \
	X(double,	"%f",	u_yaw		) \
	X(double,	"%f",	u_X			) \
	X(double,	"%f",	u_Y			) \
									  \
	X(double,	"%f",	mot_1		) \
	X(double,	"%f",	mot_2		) \
	X(double,	"%f",	mot_3		) \
	X(double,	"%f",	mot_4		) \
	X(double,	"%f",	mot_5		) \
	X(double,	"%f",	mot_6		) \
	X(double,	"%f",	vbatt		)


/*******************************************************************************
* log_entry_t
*
* Struct definition to contain a single line of the log. For each log entry
* you wish to create. Fill in an instance of this and pass to add_log_entry()
*******************************************************************************/
#define X(type, fmt, name) type name ;
typedef struct log_entry_t { LOG_TABLE } log_entry_t;
#undef X

