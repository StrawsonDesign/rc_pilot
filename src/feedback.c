/**
 * @file feedback.c
 *
 */

#include <stdio.h>
#include <math.h>
#include <rc/math/filter.h>
#include <rc/math/kalman.h>
#include <rc/math/quaternion.h>
#include <rc/math/other.h>
#include <rc/start_stop.h>
#include <rc/led.h>
#include <rc/mpu.h>
#include <rc/servo.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/bmp.h>

#include <feedback.h>
#include <rc_pilot_defs.h>
#include <setpoint_manager.h>
#include <log_manager.h>
#include <settings.h>
#include <mix.h>
#include <thrust_map.h>

#define TWO_PI (M_PI*2.0)

feedback_state_t fstate; // extern variable in feedback.h

// keep original controller gains for scaling later
static double D_roll_gain_orig, D_pitch_gain_orig, D_yaw_gain_orig, D_altitude_gain_orig;
static double dt; // controller timestep
static int num_yaw_spins;
static double last_yaw;
static double tmp;
static int last_en_alt_ctrl;

static rc_filter_t D_roll, D_pitch, D_yaw, D_batt, D_altitude, altitude_lp;
static rc_mpu_data_t mpu_data;
static rc_bmp_data_t bmp_data;

// altitude kalman filer elements
static rc_matrix_t F, G, H, Q, R, Pi;
static rc_kalman_t kf;
static rc_vector_t u,y;
static rc_filter_t acc_lp;

log_entry_t new_log;

// local functions
static void __feedback_isr(void);
static int __send_motor_stop_pulse();
static double __batt_voltage();
static int __estimate_altitude();
static int __feedback_control();
static int __feedback_state_estimate();



static void __feedback_isr(void)
{
	setpoint_manager_update();
	__feedback_state_estimate();
	__estimate_altitude();
	__feedback_control();
}


static int __send_motor_stop_pulse()
{
	int i;
	if(settings.num_rotors>8){
		printf("ERROR: set_motors_to_idle: too many rotors\n");
		return -1;
	}
	for(i=0;i<settings.num_rotors;i++){
		fstate.m[i] = -0.1;
		rc_servo_send_esc_pulse_normalized(i+1,-0.1);
	}
	return 0;
}

static double __batt_voltage()
{
	float tmp;

	tmp = rc_adc_dc_jack();
	if(tmp<3.0f) tmp = settings.v_nominal;
	return tmp;
}

static int __estimate_altitude()
{
	int i;
	double accel_vec[3];
	static int bmp_sample_counter = 0;

	// check if we need to sample BMP this loop
	if(bmp_sample_counter>=BMP_RATE_DIV){
		// perform the i2c reads to the sensor, on bad read just try later
		if(rc_bmp_read(&bmp_data)) return -1;
		bmp_sample_counter=0;
	}
	bmp_sample_counter++;

	// make copy of acceleration reading before rotating
	for(i=0;i<3;i++) accel_vec[i]=mpu_data.accel[i];
	// rotate accel vector
	rc_quaternion_rotate_vector_array(accel_vec,mpu_data.dmp_quat);

	// do first-run filter setup
	if(kf.step==0){
		kf.x_est.d[0] = bmp_data.alt_m;
		rc_filter_prefill_inputs(&acc_lp, accel_vec[2]-9.80665);
		rc_filter_prefill_outputs(&acc_lp, accel_vec[2]-9.80665);
	}

	// calculate acceleration and smooth it just a tad
	rc_filter_march(&acc_lp, accel_vec[2]-9.80665);
	u.d[0] = acc_lp.newest_output;

	// don't bother filtering Barometer, kalman will deal with that
	y.d[0] = bmp_data.alt_m;
	rc_kalman_update_lin(&kf, u, y);

	// altitude estimate
	fstate.altitude_bmp = rc_filter_march(&altitude_lp,bmp_data.alt_m);
	fstate.altitude_kf = kf.x_est.d[0];
	fstate.alt_kf_vel = kf.x_est.d[1];
	fstate.alt_kf_accel = kf.x_est.d[2];

	return 0;
}


int feedback_disarm()
{
	fstate.arm_state = DISARMED;
	// set LEDs
	rc_led_set(RC_LED_RED,1);
	rc_led_set(RC_LED_GREEN,0);
	return 0;
}


int feedback_arm()
{
	if(fstate.arm_state==ARMED){
		printf("WARNING: trying to arm when controller is already armed\n");
		return -1;
	}
	// start a new log file every time controller is armed, this may take some
	// time so do it before touching anything else
	if(settings.enable_logging) log_manager_init();
	// get the current time
	fstate.arm_time_ns = rc_nanos_since_boot();
	// reset the index
	fstate.loop_index = 0;
	// when swapping from direct throttle to altitude control, the altitude
	// controller needs to know the last throttle input for smooth transition
	//last_en_alt_ctrl = 0;
	//last_usr_thr = MIN_Z_COMPONENT;
	// yaw estimator can be zero'd too
	num_yaw_spins = 0;
	last_yaw = -mpu_data.fused_TaitBryan[TB_YAW_Z]; // minus because NED coordinates
	// zero out all filters
	rc_filter_reset(&D_roll);
	rc_filter_reset(&D_pitch);
	rc_filter_reset(&D_yaw);
	rc_filter_reset(&D_altitude);

	// prefill filters with current error
	rc_filter_prefill_inputs(&D_roll, -fstate.roll);
	rc_filter_prefill_inputs(&D_pitch, -fstate.pitch);
	// set LEDs
	rc_led_set(RC_LED_RED,0);
	rc_led_set(RC_LED_GREEN,1);
	// last thing is to flag as armed
	fstate.arm_state = ARMED;
	return 0;
}


int __init_altitude_kf()
{
//initialize altitude kalman filter and bmp sensor
	F = rc_matrix_empty();
	G = rc_matrix_empty();
	H = rc_matrix_empty();
	Q = rc_matrix_empty();
	R = rc_matrix_empty();
	Pi = rc_matrix_empty();
	u = rc_vector_empty();
	y = rc_vector_empty();
	kf = rc_kalman_empty();
	acc_lp = rc_filter_empty();
	altitude_lp = rc_filter_empty();

	int Nx = 3;
	int Ny = 1;
	int Nu = 1;
	// allocate appropirate memory for system
	rc_matrix_zeros(&F, Nx, Nx);
	rc_matrix_zeros(&G, Nx, Nu);
	rc_matrix_zeros(&H, Ny, Nx);
	rc_matrix_zeros(&Q, Nx, Nx);
	rc_matrix_zeros(&R, Ny, Ny);
	rc_matrix_zeros(&Pi, Nx, Nx);
	rc_vector_zeros(&u, Nu);
	rc_vector_zeros(&y, Ny);

// define system -DT; // accel bias
	F.d[0][0] = 1.0;
	F.d[0][1] = dt;
	F.d[0][2] = 0.0;
	F.d[1][0] = 0.0;
	F.d[1][1] = 1.0;
	F.d[1][2] = -dt; // subtract accel bias
	F.d[2][0] = 0.0;
	F.d[2][1] = 0.0;
	F.d[2][2] = 1.0; // accel bias state

	G.d[0][0] = 0.5*dt*dt;
	G.d[0][1] = dt;
	G.d[0][2] = 0.0;

	H.d[0][0] = 1.0;
	H.d[0][1] = 0.0;
	H.d[0][2] = 0.0;

	// covariance matrices
	Q.d[0][0] = 0.000000001;
	Q.d[1][1] = 0.000000001;
	Q.d[2][2] = 0.0001; // don't want bias to change too quickly
	R.d[0][0] = 1000000.0;

	// initial P, cloned from converged P while running
	Pi.d[0][0] = 1258.69;
	Pi.d[0][1] = 158.6114;
	Pi.d[0][2] = -9.9937;
	Pi.d[1][0] = 158.6114;
	Pi.d[1][1] = 29.9870;
	Pi.d[1][2] = -2.5191;
	Pi.d[2][0] = -9.9937;
	Pi.d[2][1] = -2.5191;
	Pi.d[2][2] = 0.3174;

	// initialize the kalman filter
	if(rc_kalman_alloc_lin(&kf,F,G,H,Q,R,Pi)==-1) return -1;
	// initialize the little LP filter to take out accel noise
	if(rc_filter_first_order_lowpass(&acc_lp, dt, 20*dt)) return -1;

	// initialize a LP on baromter for comparison to KF
	if(rc_filter_butterworth_lowpass(&altitude_lp, 2, dt, ALT_CUTOFF_FREQ)) return -1;

	// init barometer and read in first data
	if(rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_16))	return -1;
	if(rc_bmp_read(&bmp_data)) return -1;
	rc_filter_prefill_inputs(&altitude_lp, bmp_data.alt_m);
	rc_filter_prefill_outputs(&altitude_lp, bmp_data.alt_m);

	return 0;
}

int feedback_init()
{
	double tmp;

	// get controllers from settings
	if(settings_get_roll_controller(&D_roll)) return -1;
	if(settings_get_pitch_controller(&D_pitch)) return -1;
	if(settings_get_yaw_controller(&D_yaw)) return -1;
	if(settings_get_altitude_controller(&D_altitude)) return -1;
	dt = 1.0/settings.feedback_hz;

	if(__init_altitude_kf()) return -1;

	#ifdef DEBUG
	printf("ROLL CONTROLLER:\n");
	rc_filter_print(D_roll);
	printf("PITCH CONTROLLER:\n");
	rc_filter_print(D_pitch);
	printf("YAW CONTROLLER:\n");
	rc_filter_print(D_yaw);
	printf("ALTITUDE CONTROLLER:\n");
	rc_filter_print(D_altitude);
	#endif

	// save original gains as we will scale these by battery voltage later
	D_roll_gain_orig = D_roll.gain;
	D_pitch_gain_orig = D_pitch.gain;
	D_yaw_gain_orig = D_yaw.gain;
	D_altitude_gain_orig = D_altitude.gain;


	//enable saturation
	rc_filter_enable_saturation(&D_roll,     -1.0, 1.0);
	rc_filter_enable_saturation(&D_pitch,    -1.0, 1.0);
	rc_filter_enable_saturation(&D_yaw,      -1.0, 1.0);
	rc_filter_enable_saturation(&D_altitude, -1.0, 1.0);

	// enable soft start
	rc_filter_enable_soft_start(&D_roll, SOFT_START_SECONDS);
	rc_filter_enable_soft_start(&D_pitch, SOFT_START_SECONDS);
	rc_filter_enable_soft_start(&D_yaw, SOFT_START_SECONDS);
	rc_filter_enable_soft_start(&D_altitude, SOFT_START_SECONDS);

	// make battery filter
	rc_filter_moving_average(&D_batt, 20, 0.01);
	tmp = __batt_voltage();
	rc_filter_prefill_inputs(&D_batt, tmp);
	rc_filter_prefill_outputs(&D_batt, tmp);

	// start the IMU
	rc_mpu_config_t mpu_conf = rc_mpu_default_config();
	mpu_conf.dmp_sample_rate = settings.feedback_hz;
	mpu_conf.dmp_fetch_accel_gyro = 1;
	// optionally enbale magnetometer
	mpu_conf.enable_magnetometer = settings.enable_magnetometer;
	mpu_conf.orient = ORIENTATION_Z_UP;

	// now set up the imu for dmp interrupt operation
	printf("initializing MPU\n");
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_conf)){
		fprintf(stderr,"ERROR: in feedback_init, failed to start MPU DMP\n");
		return -1;
	}

	// make sure everything is disarmed them start the ISR
	feedback_disarm();
	fstate.initialized=1;
	rc_mpu_set_dmp_callback(__feedback_isr);

	return 0;
}


int feedback_cleanup()
{
	__send_motor_stop_pulse();
	rc_mpu_power_off();
	return 0;
}


static int __feedback_state_estimate()
{
	double tmp, yaw_reading;

	if(fstate.initialized==0){
		fprintf(stderr, "ERROR in feedback_state_estimate, feedback controller not initialized\n");
		return -1;
	}

	// collect new IMU roll/pitch data
	if(settings.enable_magnetometer){
		fstate.roll  = mpu_data.fused_TaitBryan[TB_ROLL_Y];
		fstate.pitch = mpu_data.fused_TaitBryan[TB_PITCH_X];
		yaw_reading  = mpu_data.fused_TaitBryan[TB_YAW_Z];

	}
	else{
		fstate.roll  = mpu_data.dmp_TaitBryan[TB_ROLL_Y];
		fstate.pitch = mpu_data.dmp_TaitBryan[TB_PITCH_X];
		yaw_reading  = mpu_data.dmp_TaitBryan[TB_YAW_Z];
	}

	fstate.roll_rate = mpu_data.gyro[0];
	fstate.pitch_rate = mpu_data.gyro[1];
	fstate.yaw_rate = -mpu_data.gyro[2];

	// yaw is more annoying since we have to detect spins
	// also make sign negative since NED coordinates has Z point down
	tmp = -yaw_reading + (num_yaw_spins * TWO_PI);
	//detect the crossover point at +-PI and write new value to core state
	if(tmp-last_yaw < -M_PI) num_yaw_spins++;
	else if (tmp-last_yaw > M_PI) num_yaw_spins--;
	// finally num_yaw_spins is updated and the new value can be written
	fstate.yaw = -yaw_reading + (num_yaw_spins * TWO_PI);
	last_yaw = fstate.yaw;

	// filter battery voltage.
	fstate.v_batt = rc_filter_march(&D_batt,__batt_voltage());

	return 0;
}

static int __feedback_control()
{
	int i;
	double tmp, min, max;
	double u[6], mot[8];

	// Disarm if rc_state is somehow paused without disarming the controller.
	// This shouldn't happen if other threads are working properly.
	if(rc_get_state()!=RUNNING && fstate.arm_state==ARMED){
		feedback_disarm();
	}

	// check for a tipover
	if(fabs(fstate.roll)>TIP_ANGLE || fabs(fstate.pitch)>TIP_ANGLE){
		feedback_disarm();
		printf("\n TIPOVER DETECTED \n");
	}

	// if not running or not armed, keep the motors in an idle state
	if(rc_get_state()!=RUNNING || fstate.arm_state==DISARMED){
		__send_motor_stop_pulse();
		return 0;
	}

	// We are about to start marching the individual SISO controllers forward.
	// Start by zeroing out the motors signals then add from there.
	for(i=0;i<8;i++) mot[i] = 0.0;
	for(i=0;i<6;i++) u[i] = 0.0;


	/***************************************************************************
	* Throttle/Altitude Controller
	*
	* If transitioning from direct throttle to altitude control, prefill the
	* filter with current throttle input to make smooth transition. This is also
	* true if taking off for the first time in altitude mode as arm_controller
	* sets up last_en_alt_ctrl and last_usr_thr every time controller arms
	***************************************************************************/
	// run altitude controller if enabled
	// this needs work...
	// we need to:
	//		find hover thrust and correct from there
	//		this code does not work a.t.m.
	if(setpoint.en_alt_ctrl){
		if(last_en_alt_ctrl == 0){
			setpoint.altitude = fstate.altitude_kf; // set altitude setpoint to current altitude
			rc_filter_reset(&D_altitude);
			tmp = -setpoint.Z_throttle / (cos(fstate.roll)*cos(fstate.pitch));
			rc_filter_prefill_outputs(&D_altitude, tmp);
			last_en_alt_ctrl = 1;
		}
		D_altitude.gain = D_altitude_gain_orig * settings.v_nominal/fstate.v_batt;
		tmp = rc_filter_march(&D_altitude, -setpoint.altitude+fstate.altitude_kf); //altitude is positive but +Z is down
		rc_saturate_double(&tmp, MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);
		u[VEC_Z] = tmp / cos(fstate.roll)*cos(fstate.pitch);
		mix_add_input(u[VEC_Z], VEC_Z, mot);
		last_en_alt_ctrl = 1;
	}
	// else use direct throttle
	else{

		// compensate for tilt
		tmp = setpoint.Z_throttle / (cos(fstate.roll)*cos(fstate.pitch));
		//printf("throttle: %f\n",tmp);
		rc_saturate_double(&tmp, MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);
		u[VEC_Z] = tmp;
		mix_add_input(u[VEC_Z], VEC_Z, mot);
	}

	/***************************************************************************
	* Roll Pitch Yaw controllers, only run if enabled
	***************************************************************************/
	if(setpoint.en_rpy_ctrl){
		// Roll
		mix_check_saturation(VEC_ROLL, mot, &min, &max);
		if(max>MAX_ROLL_COMPONENT)  max =  MAX_ROLL_COMPONENT;
		if(min<-MAX_ROLL_COMPONENT) min = -MAX_ROLL_COMPONENT;
		rc_filter_enable_saturation(&D_roll, min, max);
		D_roll.gain = D_roll_gain_orig * settings.v_nominal/fstate.v_batt;
		u[VEC_ROLL] = rc_filter_march(&D_roll, setpoint.roll - fstate.roll);
		mix_add_input(u[VEC_ROLL], VEC_ROLL, mot);

		// pitch
		mix_check_saturation(VEC_PITCH, mot, &min, &max);
		if(max>MAX_PITCH_COMPONENT)  max =  MAX_PITCH_COMPONENT;
		if(min<-MAX_PITCH_COMPONENT) min = -MAX_PITCH_COMPONENT;
		rc_filter_enable_saturation(&D_pitch, min, max);
		D_pitch.gain = D_pitch_gain_orig * settings.v_nominal/fstate.v_batt;
		u[VEC_PITCH] = rc_filter_march(&D_pitch, setpoint.pitch - fstate.pitch);
		mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);

		// Yaw
		// if throttle stick is down (waiting to take off) keep yaw setpoint at
		// current heading, otherwide update by yaw rate
		mix_check_saturation(VEC_YAW, mot, &min, &max);
		if(max>MAX_YAW_COMPONENT)  max =  MAX_YAW_COMPONENT;
		if(min<-MAX_YAW_COMPONENT) min = -MAX_YAW_COMPONENT;
		rc_filter_enable_saturation(&D_yaw, min, max);
		D_yaw.gain = D_yaw_gain_orig * settings.v_nominal/fstate.v_batt;
		u[VEC_YAW] = rc_filter_march(&D_yaw, setpoint.yaw - fstate.yaw);
		mix_add_input(u[VEC_YAW], VEC_YAW, mot);
	}
	// otherwise direct throttle to roll pitch yaw
	else{
		// roll
		mix_check_saturation(VEC_ROLL, mot, &min, &max);
		if(max>MAX_ROLL_COMPONENT)  max =  MAX_ROLL_COMPONENT;
		if(min<-MAX_ROLL_COMPONENT) min = -MAX_ROLL_COMPONENT;
		u[VEC_ROLL] = setpoint.roll_throttle;
		rc_saturate_double(&u[VEC_ROLL], min, max);
		mix_add_input(u[VEC_ROLL], VEC_ROLL, mot);

		// pitch
		mix_check_saturation(VEC_PITCH, mot, &min, &max);
		if(max>MAX_PITCH_COMPONENT)  max =  MAX_PITCH_COMPONENT;
		if(min<-MAX_PITCH_COMPONENT) min = -MAX_PITCH_COMPONENT;
		u[VEC_PITCH] = setpoint.pitch_throttle;
		rc_saturate_double(&u[VEC_PITCH], min, max);
		mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);

		// YAW
		mix_check_saturation(VEC_YAW, mot, &min, &max);
		if(max>MAX_YAW_COMPONENT)  max =  MAX_YAW_COMPONENT;
		if(min<-MAX_YAW_COMPONENT) min = -MAX_YAW_COMPONENT;
		u[VEC_YAW] = setpoint.yaw_throttle;
		rc_saturate_double(&u[VEC_YAW], min, max);
		mix_add_input(u[VEC_YAW], VEC_YAW, mot);
	}

	// for 6dof systems, add X and Y
	if(setpoint.en_6dof){
		// X
		mix_check_saturation(VEC_X, mot, &min, &max);
		if(max>MAX_X_COMPONENT)  max =  MAX_X_COMPONENT;
		if(min<-MAX_X_COMPONENT) min = -MAX_X_COMPONENT;
		u[VEC_X] = setpoint.X_throttle;
		rc_saturate_double(&u[VEC_X], min, max);
		mix_add_input(u[VEC_X], VEC_X, mot);

		// Y
		mix_check_saturation(VEC_Y, mot, &min, &max);
		if(max>MAX_Y_COMPONENT)  max =  MAX_Y_COMPONENT;
		if(min<-MAX_Y_COMPONENT) min = -MAX_Y_COMPONENT;
		u[VEC_Y] = setpoint.Y_throttle;
		rc_saturate_double(&u[VEC_Y], min, max);
		mix_add_input(u[VEC_Y], VEC_Y, mot);
	}

	/***************************************************************************
	* Send ESC motor signals immediately at the end of the control loop
	***************************************************************************/
	for(i=0;i<settings.num_rotors;i++){
		rc_saturate_double(&mot[i], 0.0, 1.0);
		fstate.m[i] = map_motor_signal(mot[i]);

		// NO NO NO this undoes all the fancy mixing-based saturation
		// done above, idle should be done with MAX_THRUST_COMPONENT instead
		//rc_saturate_double(&fstate.m[i], MOTOR_IDLE_CMD, 1.0);


		// final saturation just to take care of possible rounding errors
		// this should not change the values and is probably excessive
		rc_saturate_double(&fstate.m[i], 0.0, 1.0);

		// finally send pulses!
		rc_servo_send_esc_pulse_normalized(i+1,fstate.m[i]);
	}

	/***************************************************************************
	* Final cleanup, timing, and indexing
	***************************************************************************/
	// Load control inputs into cstate for viewing by outside threads
	for(i=0;i<6;i++) fstate.u[i]=u[i];
	// keep track of loops since arming
	fstate.loop_index++;
	// log us since arming, mostly for the log
	fstate.last_step_ns = rc_nanos_since_boot();


	/***************************************************************************
	* Add new log entry
	***************************************************************************/
	if(settings.enable_logging){
		new_log.loop_index	= fstate.loop_index;
		new_log.last_step_ns	= fstate.last_step_ns;
		new_log.altitude_kf	= fstate.altitude_kf;
		new_log.altitude_bmp	= fstate.altitude_bmp;
		new_log.roll		= fstate.roll;
		new_log.pitch		= fstate.pitch;
		new_log.yaw		= fstate.yaw;
		new_log.Z_throttle_sp	= setpoint.Z_throttle;
		new_log.altitude_sp	= setpoint.altitude;
		new_log.roll_sp		= setpoint.roll;
		new_log.pitch_sp	= setpoint.pitch;
		new_log.yaw_sp		= setpoint.yaw;
		new_log.v_batt		= fstate.v_batt;
		new_log.u_X		= u[VEC_Y];
		new_log.u_Y		= u[VEC_X];
		new_log.u_Z		= u[VEC_Z];
		new_log.u_roll		= u[VEC_ROLL];
		new_log.u_pitch		= u[VEC_PITCH];
		new_log.u_yaw		= u[VEC_YAW];
		new_log.mot_1		= fstate.m[0];
		new_log.mot_2		= fstate.m[1];
		new_log.mot_3		= fstate.m[2];
		new_log.mot_4		= fstate.m[3];
		new_log.mot_5		= fstate.m[4];
		new_log.mot_6		= fstate.m[5];
		add_log_entry(new_log);
	}

	return 0;
}


