/**
 * @file log_manager.c
 */

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

// to allow printf macros for multi-architecture portability
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include <rc/pthread.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#include <feedback.h>
#include <log_manager.h>
#include <rc_pilot_defs.h>
#include <setpoint_manager.h>
#include <settings.h>
#include <state_estimator.h>
#include <thread_defs.h>
#include <xbee_receive.h>

#define MAX_LOG_FILES 500
#define BUF_LEN 50

static uint64_t num_entries;  ///< number of entries logged so far
static int buffer_pos;        ///< position in current buffer
static int current_buf;       ///< 0 or 1 to indicate which buffer is being filled
static int needs_writing;     ///< flag set to 1 if a buffer is full
static FILE* log_fd;          ///< file descriptor for the log file

// array of two buffers so one can fill while writing the other to file
static log_entry_t buffer[2][BUF_LEN];

// background thread and running flag
static pthread_t log_thread;
static int logging_enabled;  // set to 0 to exit the write_thread

static int __write_header(FILE* log_fd)
{
    // always print loop index
    fprintf(log_fd, "loop_index,last_step_ns,imu_time_ns");

    if (settings.log_sensors)
    {
        fprintf(
            log_fd, ",v_batt,alt_bmp_raw,gyro_roll,gyro_pitch,gyro_yaw,accel_X,accel_Y,accel_Z");
    }

    if (settings.log_state)
    {
        fprintf(log_fd, ",roll,pitch,yaw,X,Y,Z,Xdot,Ydot,Zdot");
    }

    if (settings.log_xbee)
    {
        fprintf(log_fd,
            ",xbee_time,xbee_time_received_ns,xbee_x,xbee_y,xbee_z,xbee_qw,xbee_qx,xbee_qy,xbee_"
            "qz");
    }

    if (settings.log_throttles)
    {
        fprintf(log_fd, ",X_thrt,Y_thrt,Z_thrt,roll_thrt,pitch_thrt,yaw_thrt");
    }

    if (settings.log_setpoint)
    {
        fprintf(log_fd, ",sp_roll,sp_pitch,sp_yaw,sp_X,sp_Y,sp_Z,sp_Xdot,sp_Ydot,sp_Zdot");
    }

    if (settings.log_control_u)
    {
        fprintf(log_fd, ",u_roll,u_pitch,u_yaw,u_X,u_Y,u_Z");
    }

    if (settings.log_motor_signals && settings.num_rotors == 8)
    {
        fprintf(log_fd, ",mot_1,mot_2,mot_3,mot_4,mot_5,mot_6,mot_7,mot_8");
    }
    if (settings.log_motor_signals && settings.num_rotors == 6)
    {
        fprintf(log_fd, ",mot_1,mot_2,mot_3,mot_4,mot_5,mot_6");
    }
    if (settings.log_motor_signals && settings.num_rotors == 4)
    {
        fprintf(log_fd, ",mot_1,mot_2,mot_3,mot_4");
    }
    if (settings.log_dsm)
    {
        fprintf(log_fd, ",dsm_con");
    }
    if (settings.log_flight_mode)
    {
        fprintf(log_fd, ",flight_mode");
    }

    fprintf(log_fd, "\n");
    return 0;
}

static int __write_log_entry(FILE* log_fd, log_entry_t l)
{
    // always print loop index
    fprintf(
        log_fd, "%" PRIu64 ",%" PRIu64 ",%" PRIu64, l.loop_index, l.last_step_ns, l.imu_time_ns);

    if (settings.log_sensors)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", l.v_batt, l.alt_bmp_raw,
            l.gyro_roll, l.gyro_pitch, l.gyro_yaw, l.accel_X, l.accel_Y, l.accel_Z);
    }

    if (settings.log_state)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", l.roll, l.pitch, l.yaw,
            l.X, l.Y, l.Z, l.Xdot, l.Ydot, l.Zdot);
    }

    if (settings.log_xbee)
    {
        fprintf(log_fd, ",%" PRIu32 ",%" PRIu64, l.xbee_time, l.xbee_time_received_ns);
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", l.xbee_x, l.xbee_y, l.xbee_z,
            l.xbee_qw, l.xbee_qx, l.xbee_qy, l.xbee_qz);
    }

    if (settings.log_throttles)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", l.X_throttle, l.Y_throttle, l.Z_throttle,
            l.roll_throttle, l.pitch_throttle, l.yaw_throttle);
    }

    if (settings.log_setpoint)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", l.sp_roll, l.sp_pitch,
            l.sp_yaw, l.sp_X, l.sp_Y, l.sp_Z, l.sp_Xdot, l.sp_Ydot, l.sp_Zdot);
    }

    if (settings.log_control_u)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", l.u_roll, l.u_pitch, l.u_yaw, l.u_X,
            l.u_Y, l.u_Z);
    }

    if (settings.log_motor_signals && settings.num_rotors == 8)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", l.mot_1, l.mot_2, l.mot_3,
            l.mot_4, l.mot_5, l.mot_6, l.mot_7, l.mot_8);
    }
    if (settings.log_motor_signals && settings.num_rotors == 6)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", l.mot_1, l.mot_2, l.mot_3, l.mot_4,
            l.mot_5, l.mot_6);
    }
    if (settings.log_motor_signals && settings.num_rotors == 4)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F", l.mot_1, l.mot_2, l.mot_3, l.mot_4);
    }
    if (settings.log_dsm)
    {
        fprintf(log_fd, ",%i", l.dsm_con);
    }
    if (settings.log_flight_mode)
    {
        fprintf(log_fd, ",%i", l.flight_mode);
    }

    fprintf(log_fd, "\n");
    return 0;
}

static void* __log_manager_func(__attribute__((unused)) void* ptr)
{
    int i, buf_to_write;
    // while logging enabled and not exiting, write full buffers to disk
    while (rc_get_state() != EXITING && logging_enabled)
    {
        if (needs_writing)
        {
            // buffer to be written is opposite of one currently being filled
            if (current_buf == 0)
            {
                buf_to_write = 1;
            }
            else
            {
                buf_to_write = 0;
            }
            // write the full buffer to disk;
            for (i = 0; i < BUF_LEN; i++)
            {
                __write_log_entry(log_fd, buffer[buf_to_write][i]);
            }
            fflush(log_fd);
            needs_writing = 0;
        }
        rc_usleep(1000000 / LOG_MANAGER_HZ);
    }

    // if program is exiting or logging got disabled, write out the rest of
    // the logs that are in the buffer current being filled
    // printf("writing out remaining log file\n");
    for (i = 0; i < buffer_pos; i++)
    {
        __write_log_entry(log_fd, buffer[current_buf][i]);
    }
    fflush(log_fd);
    fclose(log_fd);

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
    if (logging_enabled)
    {
        log_manager_cleanup();
    }

    // first make sure the directory exists, make it if not
    if (stat(LOG_DIR, &st) == -1)
    {
        mkdir(LOG_DIR, 0755);
    }

    // search for existing log files to determine the next number in the series
    // TODO: more expressive log name
    for (i = 1; i <= MAX_LOG_FILES + 1; i++)
    {
        memset(&path, 0, sizeof(path));
        sprintf(path, LOG_DIR "%d.csv", i);
        // if file exists, move onto the next index
        if (stat(path, &st) == 0)
            continue;
        else
            break;
    }
    // limit number of log files
    if (i == MAX_LOG_FILES + 1)
    {
        fprintf(stderr, "ERROR: log file limit exceeded\n");
        fprintf(stderr, "delete old log files before continuing\n");
        return -1;
    }
    // create and open new file for writing
    log_fd = fopen(path, "w+");
    if (log_fd == 0)
    {
        printf("ERROR: can't open log file for writing\n");
        return -1;
    }

    // write header
    __write_header(log_fd);

    // start thread
    logging_enabled = 1;
    num_entries = 0;
    buffer_pos = 0;
    current_buf = 0;
    needs_writing = 0;

    // start logging thread
    if (rc_pthread_create(&log_thread, __log_manager_func, NULL, SCHED_FIFO, LOG_MANAGER_PRI) < 0)
    {
        fprintf(stderr, "ERROR in start_log_manager, failed to start thread\n");
        return -1;
    }
    rc_usleep(1000);
    return 0;
}

static log_entry_t __construct_new_entry()
{
    log_entry_t l;
    l.loop_index = fstate.loop_index;
    l.last_step_ns = fstate.last_step_ns;
    l.imu_time_ns = state_estimate.imu_time_ns;

    l.v_batt = state_estimate.v_batt_lp;
    l.alt_bmp_raw = state_estimate.alt_bmp_raw;
    l.gyro_roll = state_estimate.gyro[0];
    l.gyro_pitch = state_estimate.gyro[1];
    l.gyro_yaw = state_estimate.gyro[2];
    l.accel_X = state_estimate.accel[0];
    l.accel_Y = state_estimate.accel[1];
    l.accel_Z = state_estimate.accel[2];

    l.roll = state_estimate.roll;
    l.pitch = state_estimate.pitch;
    l.yaw = state_estimate.continuous_yaw;
    l.X = state_estimate.X;
    l.Y = state_estimate.Y;
    l.Z = state_estimate.Z;
    l.Xdot = state_estimate.vel_global[0];
    l.Ydot = state_estimate.vel_global[1];
    l.Zdot = state_estimate.vel_global[2];

    l.xbee_time = xbeeMsg.time;
    l.xbee_time_received_ns = state_estimate.xbee_time_received_ns;
    l.xbee_x = xbeeMsg.x;
    l.xbee_y = xbeeMsg.y;
    l.xbee_z = xbeeMsg.z;
    l.xbee_qw = xbeeMsg.qw;
    l.xbee_qx = xbeeMsg.qx;
    l.xbee_qy = xbeeMsg.qy;
    l.xbee_qz = xbeeMsg.qz;

    l.X_throttle = setpoint.X_throttle;
    l.Y_throttle = setpoint.Y_throttle;
    l.Z_throttle = setpoint.Z_throttle;
    l.roll_throttle = setpoint.roll_throttle;
    l.pitch_throttle = setpoint.pitch_throttle;
    l.yaw_throttle = setpoint.yaw_throttle;

    l.sp_roll = setpoint.roll;
    l.sp_pitch = setpoint.roll;
    l.sp_yaw = setpoint.yaw;
    l.sp_X = setpoint.X;
    l.sp_Y = setpoint.Y;
    l.sp_Z = setpoint.Z;
    l.sp_Xdot = setpoint.X_dot;
    l.sp_Ydot = setpoint.Y_dot;
    l.sp_Zdot = setpoint.Z_dot;

    l.u_roll = fstate.u[VEC_ROLL];
    l.u_pitch = fstate.u[VEC_PITCH];
    l.u_yaw = fstate.u[VEC_YAW];
    l.u_X = fstate.u[VEC_Y];
    l.u_Y = fstate.u[VEC_X];
    l.u_Z = fstate.u[VEC_Z];

    l.mot_1 = fstate.m[0];
    l.mot_2 = fstate.m[1];
    l.mot_3 = fstate.m[2];
    l.mot_4 = fstate.m[3];
    l.mot_5 = fstate.m[4];
    l.mot_6 = fstate.m[5];
    l.mot_7 = fstate.m[6];
    l.mot_8 = fstate.m[7];

    l.dsm_con = user_input.input_active;

    l.flight_mode = user_input.flight_mode;

    return l;
}

int log_manager_add_new()
{
    if (!logging_enabled)
    {
        fprintf(stderr, "ERROR: trying to log entry while logger isn't running\n");
        return -1;
    }
    if (needs_writing && buffer_pos >= BUF_LEN)
    {
        fprintf(stderr, "WARNING: logging buffer full, skipping log entry\n");
        return -1;
    }
    // add to buffer and increment counters
    buffer[current_buf][buffer_pos] = __construct_new_entry();
    buffer_pos++;
    num_entries++;
    // check if we've filled a buffer
    if (buffer_pos >= BUF_LEN)
    {
        buffer_pos = 0;     // reset buffer position to 0
        needs_writing = 1;  // flag the writer to dump to disk
        // swap buffers
        if (current_buf == 0)
        {
            current_buf = 1;
        }
        else
        {
            current_buf = 0;
        }
    }
    return 0;
}

int log_manager_cleanup()
{
    // just return if not logging
    if (logging_enabled == 0) return 0;

    // disable logging so the thread can stop and start multiple times
    // thread also exits on rc_get_state()==EXITING
    logging_enabled = 0;
    int ret = rc_pthread_timed_join(log_thread, NULL, LOG_MANAGER_TOUT);
    if (ret == 1)
    {
        fprintf(stderr, "WARNING: log_manager_thread exit timeout\n");
    }
    else if (ret == -1)
    {
        fprintf(stderr, "ERROR: failed to join log_manager thread\n");
    }
    return ret;
}
