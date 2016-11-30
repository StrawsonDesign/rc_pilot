/*******************************************************************************
* fly_logger.c
*
* James Strawson 2016
*******************************************************************************/

#include <roboticscape-usefulincludes.h>
#include <roboticscape.h>
#include "fly_types.h"
#include "fly_defs.h"

#define LOG_DIR 			"/root/fly_logs/"
#define MAX_LOG_FILES		500
#define BUF_LEN 			(SAMPLE_RATE_HZ/2)

/*******************************************************************************
* 	Global Variables
*******************************************************************************/
uint64_t num_entries;	// number of entries logged so far
int buffer_pos; 		// position in current buffer
int current_buf; 		// 0 or 1 to indicate which buffer is being filled
int needs_writing;		// flag set to 1 if a buffer is full
FILE* fd;				// file descriptor for the log file

// array of two buffers so one can fill while writing the other to file
log_entry_t buffer[2][BUF_LEN];

// background thread and running flag
pthread_t write_thread;
int logging_enabled; // set to 0 to exit the write_thread


/*******************************************************************************
* int print_entry(log_entry_t entry)
*
* write the contents of one entry to the console
*******************************************************************************/
int print_entry(log_entry_t entry){	
	#define X(type, fmt, name) printf("%s " fmt "\n", #name, entry.name);
	CORE_LOG_TABLE
	#undef X
	printf("\n");
	return 0;
}


/*******************************************************************************
* int add_log_entry(log_entry_t new_entry)
*
* quickly add new data to local buffer
*******************************************************************************/
int add_log_entry(log_entry_t new_entry){
	if(needs_writing && buffer_pos >= BUF_LEN){
		printf("WARNING: logging buffer full, skipping log entry\n")
		return -1;
	}
	if(!logging_enabled){
		printf("WARNING: trying to log entry while logger isn't running\n");
		return -1;
	}
	// add to buffer and increment counters
	buffer[current_buf][buffer_pos] = new_entry;
	buffer_pos++;
	num_entries++;

	// check if we've filled a buffer
	if(buffer_pos >= BUF_LEN){
		buffer_pos = 0; 	// reset buffer position to 0
		needs_writing = 1;	// flag the writer to dump to disk
		// swap buffers
		if(current_buf==0) current_buf=1;
		else current_buf=0;
	}
	return 0;
}

/*******************************************************************************
* int write_log_entry()
*
* append a single entry to the log file, called by log_thread_func
*******************************************************************************/
int write_log_entry(log_entry_t entry){
	#define X(type, fmt, name) fprintf(fd, fmt "," , entry.name);
    CORE_LOG_TABLE
	#undef X	
	fprintf(f, "\n");
	return 0;
}

/*******************************************************************************
* void* log_manager()
*
* Background thread that monitors the needs_writing flag and dumps a buffer 
* to the log file in one go.
*******************************************************************************/
void* log_manager(){
	int i, buf;

	while(1){
		int i,j;
		if(log->needs_writing){
			// buffer to be written is opposite of one currently being filled
			if(current_buf == 0) buf=1;
			else buf=0;
			// dump the buffer to disk;
			for(i=0;i<BUF_LEN;i++){
				write_log_entry(log_buffer[buf][i]);
			}
			fflush(fd);
			needs_writing = 0;
		}

		// if program is exiting or logging got disabled, write out the rest
		if(get_state()==EXITING || logging_enabled==0){
			for(i=0;i<num_entries;i++){
				write_log_entry(log_buffer[current_buf][i]);
			}
			fflush(fd);
			goto END;
		}
		usleep(1000000/LOGGER_HZ);
	}
END:
	fclose(fd);
	logging_enabled = 0;
	num_entries = 0;
	buffer_pos = 0; 
	current_buf = 0;
	needs_writing = 0;
	return NULL;
}


/*******************************************************************************
* int start_log_manager()
*
* Create a new csv log file and start the background thread
*******************************************************************************/
int start_log_manager(){
	int i;
	char path[100];
	DIR* dir;
	FILE *cal;
	struct stat st;

	// if the thread if running, stop before starting a new log file
	if(logging_enabled){
		printf("WARNING: log already running, starting a new one anyway.");
		stop_logger();
	}

	// first make sure the directory exists, make it if not
	dir = opendir(LOG_DIR);
	if(errno == ENOENT) mkdir(log_dir, 0777);
	else closedir(dir);

	// search for existing log files to determine the next number in the series
	for(i=1;i<=MAX_LOG_FILES+1;i++){
		memset(&dir, 0, sizeof(dir));
		sprintf(dir, LOG_DIR "%d.csv", i);
		// if file exists, move onto the next index
    	if(stat(path, &st)==0) continue;
    	else break;
	}
	if(i==MAX_LOG_FILES+1){
		printf("ERROR: log file limit exceeded\n");
		printf("delete old log files before continuing\n");
		return -1;
	}
	
	// open for writing
	cal = fopen(file_path, "w");
	if (cal == 0) {
		printf("ERROR: can't open log file for writing\n");
		return -1;
	}
	
	// write header
	#define X(type, fmt, name) fprintf(log->fd, "%s," , #name);
    CORE_LOG_TABLE
	#undef X	
	fprintf(log->fd, "\n");
	fflush(log->fd);
	
	// start thread
	logging_enabled = 1;
	struct sched_param params;
	params.sched_priority = LOG_THREAD_PRIORITY;
	pthread_setschedparam(log_thread, SCHED_FIFO, &params);
	pthread_create(&log_thread, NULL, &log_manager, NULL);

	return 0;
}

/*******************************************************************************
* stop_log_manager()
*
* finish writing remaining data to log and close it.
* return -1 if there was a timeout and the thread had to force close.
*******************************************************************************/
int stop_log_manager(){
	// if not running, just return
	if(logging_enabled == 0) return 0;

	// disable logging so the thread can stop and start multiple times
	logging_enabled = 0;
	// wait for the interrupt thread to exit
	// allow up to 1 second for thread cleanup
	struct timespec timeout;
	clock_gettime(CLOCK_REALTIME, &timeout);
	timespec_add(&timeout, LOG_MANAGER_TIMEOUT);
	int thread_err = 0;
	thread_err = pthread_timedjoin_np(log_thread, NULL, &thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: log_manager exit timeout\n");
		return -1;
	}
	return 0;
}