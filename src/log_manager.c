/**
 * @file log_manager.c
 */


#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <dirent.h>
#include <string.h>

#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/pthread.h>

#include <rc_pilot_defs.h>
#include <thread_defs.h>
#include <log_manager.h>


#define MAX_LOG_FILES	500
#define BUF_LEN		50


uint64_t num_entries;	// number of entries logged so far
int buffer_pos;		// position in current buffer
int current_buf;	// 0 or 1 to indicate which buffer is being filled
int needs_writing;	// flag set to 1 if a buffer is full
FILE* fd;		// file descriptor for the log file

// array of two buffers so one can fill while writing the other to file
log_entry_t buffer[2][BUF_LEN];

// background thread and running flag
pthread_t pthread;
int logging_enabled; // set to 0 to exit the write_thread

int print_entry(log_entry_t entry)
{
	#define X(type, fmt, name) printf("%s " fmt "\n", #name, entry.name);
	LOG_TABLE
	#undef X
	printf("\n");
	return 0;
}


int add_log_entry(log_entry_t new)
{
	if(!logging_enabled){
		fprintf(stderr,"ERROR: trying to log entry while logger isn't running\n");
		return -1;
	}
	if(needs_writing && buffer_pos >= BUF_LEN){
		fprintf(stderr,"WARNING: logging buffer full, skipping log entry\n");
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


int __write_log_entry(log_entry_t entry)
{
	#define X(type, fmt, name) fprintf(fd, fmt "," , entry.name);
	LOG_TABLE
	#undef X
	fprintf(fd, "\n");
	return 0;
}


void* __log_manager_func(__attribute__ ((unused)) void* ptr)
{
	int i, buf_to_write;
	// while logging enabled and not exiting, write full buffers to disk
	while(rc_get_state()!=EXITING && logging_enabled){
		if(needs_writing){
			// buffer to be written is opposite of one currently being filled
			if(current_buf==0) buf_to_write=1;
			else buf_to_write=0;
			// write the full buffer to disk;
			for(i=0;i<BUF_LEN;i++){
				__write_log_entry(buffer[buf_to_write][i]);
			}
			fflush(fd);
			needs_writing = 0;
		}
		rc_usleep(1000000/LOG_MANAGER_HZ);
	}

	// if program is exiting or logging got disabled, write out the rest of
	// the logs that are in the buffer current being filled
	//printf("writing out remaining log file\n");
	for(i=0;i<buffer_pos;i++){
		__write_log_entry(buffer[current_buf][i]);
	}
	fflush(fd);
	fclose(fd);
	//printf("log file closed\n");
	// zero out state
	logging_enabled = 0;
	num_entries = 0;
	buffer_pos = 0;
	current_buf = 0;
	needs_writing = 0;
	return NULL;
}


int log_manager_init()
{
	int i;
	char path[100];
	struct stat st = {0};

	// if the thread if running, stop before starting a new log file
	if(logging_enabled){
		//fprintf(stderr,"ERROR: in start_log_manager, log manager already running.\n");
		//return -1;
		log_manager_cleanup();
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
	if(rc_pthread_create(&pthread, __log_manager_func, NULL, SCHED_FIFO, LOG_MANAGER_PRI)<0){
		fprintf(stderr,"ERROR in start_log_manager, failed to start thread\n");
		return -1;
	}
	rc_usleep(1000);
	return 0;
}


int log_manager_cleanup()
{
	// just return if not logging
	if(logging_enabled==0) return 0;

	// disable logging so the thread can stop and start multiple times
	// thread also exits on rc_get_state()==EXITING
	logging_enabled=0;
	int ret = rc_pthread_timed_join(pthread,NULL,LOG_MANAGER_TOUT);
	if(ret==1) fprintf(stderr,"WARNING: log_manager_thread exit timeout\n");
	else if(ret==-1) fprintf(stderr,"ERROR: failed to join log_manager thread\n");
	return ret;
}
