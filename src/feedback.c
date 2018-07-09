/**
 * @file feedback.c
 *

 */

#include <stdio.h>
#include <math.h>
#include <rc/math/filter.h>
#include <rc/math/other.h>
#include <rc/start_stop.h>
#include <rc/led.h>
#include <rc/mpu.h>
#include <rc/servo.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/mpu.h>

#include <feedback.h>
#include <rc_pilot_defs.h>
#include <setpoint_manager.h>
#include <settings.h>
#include <mix.h>
#include <thrust_map.h>

#define TWO_PI (M_PI*2.0)

feedback_state_t fstate; // extern variable in feedback.h

// keep original controller gains for scaling later
static double D_roll_gain_orig, D_pitch_gain_orig, D_yaw_gain_orig;
static double dt; // controller timestep
static int num_yaw_spins;
static double last_yaw;
static double tmp;
static rc_filter_t D_roll, D_pitch, D_yaw, D_batt;
static rc_mpu_data_t mpu_data;

// local functions
static void __feedback_isr(void);
static int __set_motors_to_idle();
static double __batt_voltage();
static int __feedback_control();
static int __feedback_state_estimate();



static void __feedback_isr(void)
{
	setpoint_manager_update();
	__feedback_state_estimate();
	__feedback_control();
}


static int __set_motors_to_idle()
{
	int i;
	if(settings.num_rotors>8){
		printf("ERROR: set_motors_to_idle: too many rotors\n");
		return -1;
	}
	for(i=1;i<=settings.num_rotors;i++) rc_servo_send_esc_pulse_normalized(i,-0.1);
	return 0;
}

static double __batt_voltage()
{
	float tmp;

	if(settings.battery_connection==DC_BARREL_JACK){
		tmp = rc_adc_dc_jack();
	}
	else if(settings.battery_connection==BALANCE_PLUG){
		tmp = rc_adc_batt();
	}
	else{
		fprintf(stderr,"ERROR: invalid battery_connection_t\n");
		return settings.v_nominal;
	}

	if(tmp<3.0f) tmp = settings.v_nominal;
	return tmp;
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
	//if(settings.enable_logging) start_log_manager();
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




int feedback_init()
{
	double tmp;

	// get controllers from settings
	if(settings_get_roll_controller(&D_roll)) return -1;
	if(settings_get_pitch_controller(&D_pitch)) return -1;
	if(settings_get_yaw_controller(&D_yaw)) return -1;
	dt = 1.0/settings.feedback_hz;
	//printf("dt:%f", dt);

	// save original gains as we will scale these by battery voltage later
	D_roll_gain_orig = D_roll.gain;
	D_pitch_gain_orig = D_pitch.gain;
	D_yaw_gain_orig = D_yaw.gain;


	//enable saturation
	rc_filter_enable_saturation(&D_roll,  -1.0, 1.0);
	rc_filter_enable_saturation(&D_pitch,  -1.0, 1.0);
	rc_filter_enable_saturation(&D_yaw, -1.0, 1.0);

	// enable soft start
	rc_filter_enable_soft_start(&D_roll, SOFT_START_SECONDS);
	rc_filter_enable_soft_start(&D_pitch, SOFT_START_SECONDS);
	rc_filter_enable_soft_start(&D_yaw, SOFT_START_SECONDS);

	// make battery filter
	rc_filter_moving_average(&D_batt, 20, 0.01);
	tmp = __batt_voltage();
	rc_filter_prefill_inputs(&D_batt, tmp);
	rc_filter_prefill_outputs(&D_batt, tmp);

	// start the IMU
	rc_mpu_config_t conf = rc_mpu_default_config();
	conf.dmp_sample_rate = settings.feedback_hz;
	conf.enable_magnetometer = 1;
	conf.orient = ORIENTATION_Z_UP;

	// now set up the imu for dmp interrupt operation
	printf("initializing MPU\n");
	if(rc_mpu_initialize_dmp(&mpu_data, conf)){
		fprintf(stderr,"ERROR: in feedback_init, failed to start MPU DMP\n");
		return -1;
	}

	// make sure everything is disarmed them start the ISR
	feedback_disarm();
	fstate.initialized=1;  //pg
	rc_mpu_set_dmp_callback(__feedback_isr);

	return 0;
}


int feedback_cleanup()
{
	__set_motors_to_idle();
	rc_mpu_power_off();
	return 0;
}


static int __feedback_state_estimate()
{
	double tmp;

	if(fstate.initialized==0){
		fprintf(stderr, "ERROR in feedback_state_estimate, feedback controller not initialized\n");
		return -1;
	}

	// collect new IMU roll/pitch data

	// We don't want to use magnetometer at the moment...
	// should add option to use this to the config file
	// and switch here

	//fstate.roll   = mpu_data.fused_TaitBryan[TB_ROLL_Y];
	//fstate.pitch  = mpu_data.fused_TaitBryan[TB_PITCH_X];

	fstate.roll   = mpu_data.dmp_TaitBryan[TB_ROLL_Y];
	fstate.pitch  = mpu_data.dmp_TaitBryan[TB_PITCH_X];

	// yaw is more annoying since we have to detect spins
	// also make sign negative since NED coordinates has Z point down
	//tmp = -mpu_data.TaitBryan[TB_YAW_Z];// + (num_yaw_spins * TWO_PI);
	// detect the crossover point at +-PI and write new value to core state
	//if(tmp-last_yaw < -M_PI) num_yaw_spins++;
	//else if (tmp-last_yaw > M_PI) num_yaw_spins--;
	// finally num_yaw_spins is updated and the new value can be written
	//fstate.yaw = mpu_data.dmp_TaitBryan[TB_YAW_Z]; + (num_yaw_spins * TWO_PI);
	//last_yaw = fstate.yaw;
	
	// For the moment use raw yaw data
	fstate.yaw = mpu_data.dmp_TaitBryan[TB_YAW_Z];

	// filter battery voltage.
	fstate.v_batt = rc_filter_march(&D_batt,__batt_voltage());

	// TODO: altitude estimate
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
		__set_motors_to_idle();
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
	// // run altitude controller if enabled
	// if(setpoint.en_alt_ctrl){
	// 	if(last_en_alt_ctrl == 0){
	// 		setpoint.altitude = fstate.alt; // set altitude setpoint to current altitude
	// 		rc_reset_filter(&D0);
	// 		prefill_filter_outputs(&D0,last_usr_thr);
	// 		last_en_alt_ctrl = 1;
	// 	}
	// 	setpoint.altitude += setpoint.altitude_rate*DT;
	// 	saturate_float(&setpoint.altitude, fstate.alt-ALT_BOUND_D, fstate.alt+ALT_BOUND_U);
	// 	D0.gain = D0_GAIN * V_NOMINAL/fstate.vbatt;
	// 	tmp = march_filter(&D0, setpoint.altitude-fstate.alt);
	// 	u[VEC_Z] = tmp / cos(fstate.roll)*cos(fstate.pitch);
	// 	saturate_float(&u[VEC_Z], MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);
	// 	mix_add_input(u[VEC_Z], VEC_Z, mot);
	// 	last_en_alt_ctrl = 1;
	// }
	// // else use direct throttle
	// else{

		// compensate for tilt
		tmp = setpoint.Z_throttle / (cos(fstate.roll)*cos(fstate.pitch));
		rc_saturate_double(&tmp, MIN_Z_COMPONENT, MAX_Z_COMPONENT);
		u[VEC_Z] = tmp;
		mix_add_input(u[VEC_Z], VEC_Z, mot);
	//}

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
	// otherwise direct throttle
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
		u[VEC_PITCH] = setpoint.roll_throttle;
		rc_saturate_double(&u[VEC_PITCH], min, max);
		mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);

		// YAW
		mix_check_saturation(VEC_ROLL, mot, &min, &max);
		if(max>MAX_YAW_COMPONENT)  max =  MAX_YAW_COMPONENT;
		if(min<-MAX_YAW_COMPONENT) min = -MAX_YAW_COMPONENT;
		u[VEC_YAW] = setpoint.yaw_throttle;
		rc_saturate_double(&u[VEC_YAW], min, max);
		mix_add_input(u[VEC_YAW], VEC_YAW, mot);

		u[VEC_ROLL]	= 0.0;
		u[VEC_PITCH]	= 0.0;
		u[VEC_YAW]	= 0.0;
	}

	/***************************************************************************
	* Send ESC motor signals immediately at the end of the control loop
	***************************************************************************/
	for(i=0;i<settings.num_rotors;i++){
		rc_saturate_double(&mot[i], 0.0, 1.0);
		fstate.m[i] = map_motor_signal(mot[i]);
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
	// if(settings.enable_logging){
	// 	new_log.loop_index	= fstate.loop_index;
	// 	new_log.last_step_us	= fstate.last_step_us;
	// 	new_log.altitude	= fstate.altitude;
	// 	new_log.roll		= fstate.roll;
	// 	new_log.pitch		= fstate.pitch;
	// 	new_log.yaw		= fstate.yaw;
	// 	new_log.v_batt		= fstate.v_batt;
	// 	new_log.u_X		= u[VEC_Y];
	// 	new_log.u_Y		= u[VEC_X];
	// 	new_log.u_Z		= u[VEC_Z];
	// 	new_log.u_roll		= u[VEC_ROLL];
	// 	new_log.u_pitch		= u[VEC_PITCH];
	// 	new_log.u_yaw		= u[VEC_YAW];
	// 	new_log.mot_1		= fstate.m[0];
	// 	new_log.mot_2		= fstate.m[1];
	// 	new_log.mot_3		= fstate.m[2];
	// 	new_log.mot_4		= fstate.m[3];
	// 	new_log.mot_5		= fstate.m[4];
	// 	new_log.mot_6		= fstate.m[5];
	// 	add_log_entry(new_log);
	// }

	return 0;
}


