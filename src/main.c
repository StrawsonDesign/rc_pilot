/**
 * @file main.c
 *
 * see README.txt for description and use
 **/

#include <getopt.h>
#include <stdio.h>
#include <unistd.h>

#include <rc/adc.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/cpu.h>
#include <rc/dsm.h>
#include <rc/led.h>
#include <rc/mpu.h>
#include <rc/servo.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#include <input_manager.h>
#include <log_manager.h>
#include <mix.h>
#include <printf_manager.h>
#include <setpoint_manager.h>
#include <settings.h>  // contains extern settings variable
#include <state_estimator.h>
#include <thrust_map.h>
#include <xbee_receive.h>

#define FAIL(str)                       \
    fprintf(stderr, str);               \
    rc_led_set(RC_LED_GREEN, 0);        \
    rc_led_blink(RC_LED_RED, 8.0, 2.0); \
    return -1;

void print_usage()
{
    printf("\n");
    printf(" Options\n");
    printf(" -s {settings file} Specify settings file to use\n");
    printf(" -h                 Print this help message\n");
    printf("\n");
    printf("Some example settings files are included with the\n");
    printf("source code. You must specify the location of one of these\n");
    printf("files or ideally the location of your own settings file.\n");
    printf("\n");
}

/**
 * temporary check for dsm calibration until I add this to librobotcontrol
 */
int __rc_dsm_is_calibrated()
{
    if (!access("/var/lib/robotcontrol/dsm.cal", F_OK))
        return 1;
    else
        return 0;
}

/**
 * If the user holds the pause button for 2 seconds, set state to exiting which
 * triggers the rest of the program to exit cleanly.
 */
void on_pause_press()
{
    int i = 0;
    const int quit_check_us = 100000;
    const int samples = 2000000 / quit_check_us;

    // toggle betewen paused and running modes
    if (rc_get_state() == RUNNING)
    {
        rc_set_state(PAUSED);
        printf("PAUSED\n");
    }
    else if (rc_get_state() == PAUSED)
    {
        rc_set_state(RUNNING);
        printf("RUNNING\n");
    }
    fflush(stdout);

    // now keep checking to see if the button is still held down
    for (i = 0; i < samples; i++)
    {
        rc_usleep(quit_check_us);
        if (rc_button_get_state(RC_BTN_PIN_PAUSE) == RC_BTN_STATE_RELEASED)
        {
            return;
        }
    }
    printf("long press detected, shutting down\n");
    rc_set_state(EXITING);
    return;
}

/**
 * @brief      Interrupt service routine for IMU
 *
 * This is called every time the Invensense IMU has new data
 */
static void __imu_isr(void)
{
    // Log time that isr occurs
    state_estimate.imu_time_ns = rc_nanos_since_boot();

    setpoint_manager_update();
    state_estimator_march();
    feedback_march();

    if (settings.enable_logging) log_manager_add_new();

    state_estimator_jobs_after_feedback();
}

/**
 * Initialize the IMU, start all the threads, and wait until something triggers
 * a shut down by setting the RC state to EXITING.
 *
 * @return     0 on success, -1 on failure
 */
int main(int argc, char* argv[])
{
    int c;
    char* settings_file_path = NULL;

    // parse arguments
    opterr = 0;
    while ((c = getopt(argc, argv, "s:h")) != -1)
    {
        switch (c)
        {
            // settings file option
            case 's':
                settings_file_path = optarg;
                printf("User specified settings file:\n%s\n", settings_file_path);
                break;

            // help mode
            case 'h':
                print_usage();
                return 0;

            default:
                printf("\nInvalid Argument \n");
                print_usage();
                return -1;
        }
    }

    // settings file option is mandatory
    if (settings_file_path == NULL)
    {
        print_usage();
        return -1;
    }

    // first things first, load settings which may be used during startup
    if (settings_load_from_file(settings_file_path) < 0)
    {
        fprintf(stderr, "ERROR: failed to load settings\n");
        return -1;
    }
    printf("Loaded settings: %s\n", settings.name);

    // before touching hardware, make sure another instance isn't running
    // return value -3 means a root process is running and we need more
    // privileges to stop it.
    if (rc_kill_existing_process(2.0) == -3) return -1;

    // start with both LEDs off
    if (rc_led_set(RC_LED_GREEN, 0) == -1)
    {
        fprintf(stderr, "ERROR in main(), failed to set RC_LED_GREEN\n");
        return -1;
    }
    if (rc_led_set(RC_LED_RED, 0) == -1)
    {
        fprintf(stderr, "ERROR in main() failed to set RC_LED_RED\n");
        return -1;
    }

    // make sure IMU is calibrated
    if (!rc_mpu_is_gyro_calibrated())
    {
        FAIL("ERROR, must calibrate gyroscope with rc_calibrate_gyro first\n")
    }
    if (!rc_mpu_is_accel_calibrated())
    {
        FAIL("ERROR, must calibrate accelerometer with rc_calibrate_accel first\n")
    }
    if (settings.enable_magnetometer && !rc_mpu_is_gyro_calibrated())
    {
        FAIL("ERROR, must calibrate magnetometer with rc_calibrate_mag first\n")
    }
    if (!__rc_dsm_is_calibrated())
    {
        FAIL("ERROR, must calibrate DSM with rc_calibrate_dsm first\n")
    }

    // turn cpu freq to max for most consistent performance and lowest
    // latency servicing the IMU's interrupt service routine
    // this also serves as an initial check for root access which is needed
    // by the PRU later. PRU root acces might get resolved in the future.
    if (rc_cpu_set_governor(RC_GOV_PERFORMANCE) < 0)
    {
        FAIL("WARNING, can't set CPU governor, need to run as root\n")
    }

    // do initialization not involving threads
    printf("initializing thrust map\n");
    if (thrust_map_init(settings.thrust_map) < 0)
    {
        FAIL("ERROR: failed to initialize thrust map\n")
    }
    printf("initializing mixing matrix\n");
    if (mix_init(settings.layout) < 0)
    {
        FAIL("ERROR: failed to initialize mixing matrix\n")
    }
    printf("initializing setpoint_manager\n");
    if (setpoint_manager_init() < 0)
    {
        FAIL("ERROR: failed to initialize setpoint_manager\n")
    }

    // initialize cape hardware, this prints an error itself if unsuccessful
    printf("initializing servos\n");
    if (rc_servo_init() == -1)
    {
        FAIL("ERROR: failed to initialize servos, probably need to run as root\n")
    }
    printf("initializing adc\n");
    if (rc_adc_init() == -1)
    {
        FAIL("ERROR: failed to initialize ADC")
    }

    // start signal handler so threads can exit cleanly
    printf("initializing signal handler\n");
    if (rc_enable_signal_handler() < 0)
    {
        FAIL("ERROR: failed to complete rc_enable_signal_handler\n")
    }

    // start threads
    printf("initializing DSM and input_manager\n");
    if (input_manager_init() < 0)
    {
        FAIL("ERROR: failed to initialize input_manager\n")
    }

    // initialize buttons and Assign functions to be called when button
    // events occur
    if (rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH, RC_BTN_DEBOUNCE_DEFAULT_US))
    {
        FAIL("ERROR: failed to init buttons\n")
    }
    rc_button_set_callbacks(RC_BTN_PIN_PAUSE, on_pause_press, NULL);

    // initialize log_manager if enabled in settings
    if (settings.enable_logging)
    {
        printf("initializing log manager");
        if (log_manager_init() < 0)
        {
            FAIL("ERROR: failed to initialize log manager\n")
        }
    }

    // start barometer, must do before starting state estimator
    printf("initializing Barometer\n");
    if (rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_16))
    {
        FAIL("ERROR: failed to initialize barometer\n")
    }

    // set up state estimator
    printf("initializing state_estimator\n");
    if (state_estimator_init() < 0)
    {
        FAIL("ERROR: failed to init state_estimator")
    }

    // set up XBEE serial link
    printf("initializing xbee serial link.\n");
    if (XBEE_init() < 0)
    {
        FAIL("ERROR: failed to init xbee serial link")
    }

    // set up feedback controller
    printf("initializing feedback controller\n");
    if (feedback_init() < 0)
    {
        FAIL("ERROR: failed to init feedback controller")
    }

    // start the IMU
    rc_mpu_config_t mpu_conf = rc_mpu_default_config();
    mpu_conf.i2c_bus = I2C_BUS;
    mpu_conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
    mpu_conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
    mpu_conf.dmp_sample_rate = FEEDBACK_HZ;
    mpu_conf.dmp_fetch_accel_gyro = 1;
    // mpu_conf.orient = ORIENTATION_Z_UP;
    mpu_conf.dmp_interrupt_sched_policy = SCHED_FIFO;
    mpu_conf.dmp_interrupt_priority = IMU_PRIORITY;

    // optionally enbale magnetometer
    mpu_conf.enable_magnetometer = settings.enable_magnetometer;

    // now set up the imu for dmp interrupt operation
    printf("initializing MPU\n");
    if (rc_mpu_initialize_dmp(&mpu_data, mpu_conf))
    {
        fprintf(stderr, "ERROR: failed to start MPU DMP\n");
        return -1;
    }

    // final setup
    if (rc_make_pid_file() != 0)
    {
        FAIL("ERROR: failed to make a PID file\n")
    }

    // make sure everything is disarmed them start the ISR
    feedback_disarm();
    printf("waiting for dmp to settle...\n");
    fflush(stdout);
    rc_usleep(3000000);
    if (rc_mpu_set_dmp_callback(__imu_isr) != 0)
    {
        FAIL("ERROR: failed to set dmp callback function\n")
    }

    // start printf_thread if running from a terminal
    // if it was started as a background process then don't bother

    if (isatty(fileno(stdout)))
    {
        printf("initializing printf manager\n");
        if (printf_init() < 0)
        {
            FAIL("ERROR: failed to initialize printf_manager\n")
        }
    }

    // set state to running and chill until something exits the program
    rc_set_state(RUNNING);
    while (rc_get_state() != EXITING)
    {
        usleep(50000);
    }

    // some of these, like printf_manager and log_manager, have cleanup
    // functions that can be called even if not being used. So just call all
    // cleanup functions here.
    printf("cleaning up\n");
    rc_mpu_power_off();
    feedback_cleanup();
    input_manager_cleanup();
    setpoint_manager_cleanup();
    printf_cleanup();
    log_manager_cleanup();

    // turn off red LED and blink green to say shut down was safe
    rc_led_set(RC_LED_RED, 0);
    rc_led_blink(RC_LED_GREEN, 8.0, 2.0);
    return 0;
}
