/*******************************************************************************
* fly_logger.c
*
* James Strawson 2016
*******************************************************************************/

#define _GNU_SOURCE
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <roboticscape.h>
#include <dirent.h>
#include <time.h>
#include <string.h>
#include "fly_types.h"
#include "fly_defs.h"
#include "fly_function_declarations.h"


#define LOG_DIR 			"/root/fly_logs/"
#define MAX_LOG_FILES		500
#define BUF_LEN 			50

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
pthread_t log_thread;
int logging_enabled; // set to 0 to exit the write_thread


/*******************************************************************************
* int print_entry(log_entry_t entry)
*
* write the contents of one entry to the console
*******************************************************************************/
int print_entry(log_entry_t entry){	
	#define X(type, fmt, name) printf("%s " fmt "\n", #name, entry.name);
	LOG_TABLE
	#undef X
	printf("\n");
	return 0;
}


/*******************************************************************************
* int add_log_entry(log_entry_t new)
*
* quickly add new data to local buffer
*******************************************************************************/
int add_log_entry(log_entry_t new){
	if(!logging_enabled){
		printf("WARNING: trying to log entry while logger isn't running\n");
		return -1;
	}
	if(needs_writing && buffer_pos >= BUF_LEN){
		printf("WARNING: logging buffer full, skipping log entry\n");
		return -1;
	}
	// add to buffer and increment counters
	buffer[current_buf][buffer_pos] = new;
	buffer_pos++;
	num_entries++;
	// check if we've filled a buffer
	if(buffer_pos >= BUF_LEN){
		buffer_pos = 0;		// reset buffer position to 0
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
    LOG_TABLE
	#undef X	
	fprintf(fd, "\n");
	return 0;
}

/*******************************************************************************
* void* log_manager()
*
* Background thread that monitors the needs_writing flag and dumps a buffer 
* to the log file in one go.
*******************************************************************************/
void* log_manager(){
	int i, buf_to_write;
	// while logging enabled and not exiting, write full buffers to disk
	while(rc_get_state()!=EXITING && logging_enabled){
		if(needs_writing){
			// buffer to be written is opposite of one currently being filled
			if(current_buf==0) buf_to_write=1;
			else buf_to_write=0;
			// write the full buffer to disk;
			for(i=0;i<BUF_LEN;i++){
				write_log_entry(buffer[buf_to_write][i]);
			}
			fflush(fd);
			needs_writing = 0;
		}
		usleep(1000000/LOG_MANAGER_HZ);
	}

	// if program is exiting or logging got disabled, write out the rest of
	// the logs that are in the buffer current being filled
	for(i=0;i<num_entries;i++){
		write_log_entry(buffer[current_buf][i]);
	}
	fflush(fd);
	fclose(fd);
	// zero out state
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
	struct stat st = {0};

	// if the thread if running, stop before starting a new log file
	if(logging_enabled){
		printf("WARNING: log already running, starting a new one anyway.");
		stop_log_manager();
	}

	// make sure the previous log thread has stopped
	struct timespec timeout;
	clock_gettime(CLOCK_REALTIME, &timeout);
	timespec_add(&timeout, LOG_MANAGER_TIMEOUT);
	int thread_err = 0;
	thread_err = pthread_timedjoin_np(log_thread, NULL, &timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: log_manager exit timeout\n");
		return -1;
	}

	// first make sure the directory exists, make it if not
	if (stat(LOG_DIR, &st) == -1) {
		mkdir(LOG_DIR, 0755);
	}

	// search for existing log files to determine the next number in the series
	for(i=1;i<=MAX_LOG_FILES+1;i++){
		memset(&path, 0, sizeof(path));
		sprintf(path, LOG_DIR "%d.csv", i);
		// if file exists, move onto the next index
		if(stat(path, &st)==0) continue;
		else break;
	}
	// limit number of log files
	if(i==MAX_LOG_FILES+1){
		printf("ERROR: log file limit exceeded\n");
		printf("delete old log files before continuing\n");
		return -1;
	}
	// create and open new file for writing
	fd = fopen(path, "w+");
	if(fd == 0) {
		printf("ERROR: can't open log file for writing\n");
		return -1;
	}
	// write header
	#define X(type, fmt, name) fprintf(fd, "%s," , #name);
	LOG_TABLE
	#undef X	
	fprintf(fd, "\n");
	fflush(fd);
	
	// start thread
	logging_enabled = 1;
	num_entries = 0;
	buffer_pos = 0; 
	current_buf = 0;
	needs_writing = 0;

	// start logging thread
	pthread_create(&log_thread, NULL, &log_manager, NULL);
	struct sched_param params = {LOG_MANAGER_PRIORITY};
	pthread_setschedparam(log_thread, SCHED_FIFO, &params);
	usleep(1000);
	return 0;
}

/*******************************************************************************
* stop_log_manager()
*
* finish writing remaining data to log and close it.
* return -1 if there was a timeout and the thread had to force close.
*******************************************************************************/
int stop_log_manager(){
	// disable logging so the thread can stop and start multiple times
	logging_enabled = 0;
	return 0;
}