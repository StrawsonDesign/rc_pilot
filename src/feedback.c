/**
 * @file feedback.c
 *

 */

#include <stdio.h>
#include <math.h>
#include <rc/math/filter.h>
#include <rc/start_stop.h>
#include <rc/led.h>
#include <rc/mpu.h>
#include <rc/servo.h>

#include <feedback.h>
#include <fly_defs.h>
#include <log_manager.h>
#include <mixing_matrix.h>

#define TWO_PI (M_PI*2.0)

// pointers to outside structs
setpoint_t* sp;
cstate_t* cs;
rc_mpu_data_t* imu;
fly_settings_t* set;

// local arm state, set this from outside with arm/disarm_controller
arm_state_t arm_state;

// discrete controllers
rc_filter_t D_roll, D_pitch, D_yaw;

// keep original controller gains for scaling later
float D_roll_gain_orig, D_pitch_gain_orig, D_yaw_gain_orig;

// one log entry, passed to log manager if logging enabled
log_entry_t new_log;

// altitude controller need setup if being turned on mid flight
// so keep track of last state to detect changes.
int last_en_alt_ctrl;
float last_usr_thr;

// other
float dt; // controller timestep
int num_yaw_spins;
int last_yaw;
float u[6], mot[8], tmp;
uint64_t start_time_us;


// Local functions only for use in this c file
int set_motors_to_idle();
void feedback_controller(); // ISR


int disarm_controller()
{
	arm_state = DISARMED;
	rc_set_led(RED,1);
	rc_set_led(GREEN,0);
	return 0;
}


int arm_controller()
{
	if(arm_state==ARMED){
		printf("WARNING: trying to arm when controller is already armed\n");
		return -1;
	}
	// start a new log file every time controller is armed, this may take some
	// time so do it before touching anything else
	if(set->enable_logging) start_log_manager();
	// get the current time
	start_time_us = rc_nanos_since_epoch()/1000;
	// reset the index
	cs->loop_index = 0;
	// when swapping from direct throttle to altitude control, the altitude
	// controller needs to know the last throttle input for smooth transition
	last_en_alt_ctrl = 0;
	last_usr_thr = MIN_Z_COMPONENT;
	// yaw estimator can be zero'd too
	num_yaw_spins = 0;
	last_yaw = -imu->fused_TaitBryan[TB_YAW_Z]; // minus because NED coordinates
	// zero out all filters
	rc_reset_filter(&D_roll);
	rc_reset_filter(&D_pitch);
	rc_reset_filter(&D_yaw);
	// prefill filters with current error
	rc_prefill_filter_inputs(&D_roll, -cs->roll);
	rc_prefill_filter_inputs(&D_roll, -cs->pitch);
	// set LEDs
	rc_set_led(RED,0);
	rc_set_led(GREEN,1);
	// last thing is to flag as armed
	arm_state = ARMED;
	return 0;
}


arm_state_t get_controller_arm_state()
{
	return arm_state;
}


int init_controller(cstate_t* cstate, setpoint_t* setpoint, \
			rc_imu_data_t* imu_data, fly_settings_t* settings)
{

	// make local copies of pointers to global structs
	cs = cstate;
	sp = setpoint;
	imu = imu_data;
	set = settings;

	// get controllers from settings
	if(get_json_roll_controller(&D_roll)) return -1;
	if(get_json_pitch_controller(&D_pitch)) return -1;
	if(get_json_yaw_controller(&D_yaw)) return -1;
	dt = 1.0/set->feedback_hz;

	// save original gains as we will scale these by battery voltage later
	D_roll_gain_orig = D_roll.gain;
	D_pitch_gain_orig = D_pitch.gain;
	D_yaw_gain_orig = D_yaw.gain;

	// enable soft start
	rc_enable_soft_start(&D_roll, SOFT_START_SECONDS);
	rc_enable_soft_start(&D_pitch, SOFT_START_SECONDS);
	rc_enable_soft_start(&D_yaw, SOFT_START_SECONDS);

	// make sure everything is disarmed them start the ISR
	disarm_controller();
	rc_set_imu_interrupt_func(&feedback_controller);
	return 0;
}



int set_motors_to_idle()
{
	int i;
	if(set->num_rotors>8){
		printf("ERROR: set_motors_to_idle: too many rotors\n");
		return -1;
	}
	for(i=1;i<=set->num_rotors;i++) rc_servo_send_esc_pulse_normalized(i,-0.1);
	return 0;
}


void feedback_controller()
{
	int i;
	float tmp, min, max;
	float new_mot[8];

	/***************************************************************************
	*	STATE_ESTIMATION
	*	read sensors and compute the state regardless of if the controller
	*	is ARMED or DISARMED
	***************************************************************************/
	// collect new IMU roll/pitch data
	cs->roll   = imu->fused_TaitBryan[TB_ROLL_Y];
	cs->pitch  = imu->fused_TaitBryan[TB_PITCH_X];

	// yaw is more annoying since we have to detect spins
	// also make sign negative since NED coordinates has Z point down
	tmp = -imu->fused_TaitBryan[TB_YAW_Z] + (num_yaw_spins * TWO_PI);
	// detect the crossover point at +-PI and write new value to core state
	if(tmp-last_yaw < -M_PI) num_yaw_spins++;
	else if (tmp-last_yaw > M_PI) num_yaw_spins--;
	// finally num_yaw_spins is updated and the new value can be written
	cs->yaw = imu->fused_TaitBryan[TB_YAW_Z] + (num_yaw_spins * TWO_PI);
	last_yaw = cs->yaw;

	// TODO: altitude estimate

	/***************************************************************************
	* Now check for all conditions that prevent normal running
	***************************************************************************/
	// Disarm if rc_state is somehow paused without disarming the controller.
	// This shouldn't happen if other threads are working properly.
	if(rc_get_state()!=RUNNING && arm_state==ARMED){
		disarm_controller();
	}

	// check for a tipover
	if(fabs(cs->roll)>TIP_ANGLE || fabs(cs->pitch)>TIP_ANGLE){
		disarm_controller();
		printf("\n TIPOVER DETECTED \n");
	}

	/***************************************************************************
	* if not running or not armed, keep the motors in an idle state
	***************************************************************************/
	if(rc_get_state()!=RUNNING || arm_state==DISARMED){
		set_motors_to_idle();
		return;
	}

	/***************************************************************************
	* We are about to start marching the individual SISO controllers forward.
	* Start by zeroing out the motors signals then add from there.
	***************************************************************************/
	for(i=0;i<set->num_rotors;i++) mot[i] = 0.0;


	/***************************************************************************
	* Throttle/Altitude Controller
	*
	* If transitioning from direct throttle to altitude control, prefill the
	* filter with current throttle input to make smooth transition. This is also
	* true if taking off for the first time in altitude mode as arm_controller
	* sets up last_en_alt_ctrl and last_usr_thr every time controller arms
	***************************************************************************/
	// // run altitude controller if enabled
	// if(sp->en_alt_ctrl){
	// 	if(last_en_alt_ctrl == 0){
	// 		sp->altitude = cs->alt; // set altitude setpoint to current altitude
	// 		rc_reset_filter(&D0);
	// 		prefill_filter_outputs(&D0,last_usr_thr);
	// 		last_en_alt_ctrl = 1;
	// 	}
	// 	sp->altitude += sp->altitude_rate*DT;
	// 	saturate_float(&sp->altitude, cs->alt-ALT_BOUND_D, cs->alt+ALT_BOUND_U);
	// 	D0.gain = D0_GAIN * V_NOMINAL/cs->vbatt;
	// 	tmp = march_filter(&D0, sp->altitude-cs->alt);
	// 	u[VEC_Z] = tmp / cos(cs->roll)*cos(cs->pitch);
	// 	saturate_float(&u[VEC_Z], MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);
	// 	add_mixed_input(u[VEC_Z], VEC_Z, mot);
	// 	last_en_alt_ctrl = 1;
	// }
	// // else use direct throttle
	// else{

	// compensate for tilt
	tmp = sp->Z_throttle / (cos(cs->roll)*cos(cs->pitch));
	rc_saturate_float(&tmp, -MIN_Z_COMPONENT, -MAX_Z_COMPONENT);
	u[VEC_Z] = tmp;
	add_mixed_input(u[VEC_Z], VEC_Z, mot);
	// save throttle in case of transition to altitude control
	last_usr_thr = sp->Z_throttle;
	last_en_alt_ctrl = 0;

	/***************************************************************************
	* Roll Pitch Yaw controllers, only run if enabled
	***************************************************************************/
	if(sp->en_rpy_ctrl){
		// Roll
		check_channel_saturation(VEC_ROLL, mot, &min, &max);
		if(max>MAX_ROLL_COMPONENT)  max =  MAX_ROLL_COMPONENT;
		if(min<-MAX_ROLL_COMPONENT) min = -MAX_ROLL_COMPONENT;
		rc_enable_saturation(&D_roll, min, max);
		D_roll.gain = D_roll_gain_orig * set->v_nominal/cs->v_batt;
		u[VEC_ROLL] = rc_march_filter(&D_roll, sp->roll - cs->roll);
		add_mixed_input(u[VEC_ROLL], VEC_ROLL, mot);

		// pitch
		check_channel_saturation(VEC_PITCH, mot, &min, &max);
		if(max>MAX_PITCH_COMPONENT)  max =  MAX_PITCH_COMPONENT;
		if(min<-MAX_PITCH_COMPONENT) min = -MAX_PITCH_COMPONENT;
		rc_enable_saturation(&D_pitch, min, max);
		D_pitch.gain = D_pitch_gain_orig * set->v_nominal/cs->v_batt;
		u[VEC_PITCH] = rc_march_filter(&D_pitch, sp->pitch - cs->pitch);
		add_mixed_input(u[VEC_PITCH], VEC_PITCH, mot);

		// Yaw
		// if throttle stick is down (waiting to take off) keep yaw setpoint at
		// current heading, otherwide update by yaw rate
		sp->yaw += dt*sp->yaw_rate;
		check_channel_saturation(VEC_YAW, mot, &min, &max);
		if(max>MAX_YAW_COMPONENT)  max =  MAX_YAW_COMPONENT;
		if(min<-MAX_YAW_COMPONENT) min = -MAX_YAW_COMPONENT;
		rc_enable_saturation(&D_yaw, min, max);
		D_yaw.gain = D_yaw_gain_orig * set->v_nominal/cs->v_batt;
		u[VEC_YAW] = rc_march_filter(&D_yaw, sp->yaw - cs->yaw);
		add_mixed_input(u[VEC_YAW], VEC_YAW, mot);
	}
	else{
		u[VEC_ROLL]	= 0.0;
		u[VEC_PITCH]	= 0.0;
		u[VEC_YAW]	= 0.0;
	}

	/***************************************************************************
	* X (Side) and Y (Forward) inputs, only when 6dof is enabled
	***************************************************************************/
	if(sp->en_6dof){
		// Y (sideways)
		u[VEC_Y] = sp->X_throttle;
		check_channel_saturation(VEC_Y, mot, &min, &max);
		if(max>MAX_X_COMPONENT)  max =  MAX_X_COMPONENT;
		if(min<-MAX_X_COMPONENT) min = -MAX_X_COMPONENT;
		rc_saturate_float(&u[VEC_Y], min, max);
		// add mixed components to the motors
		add_mixed_input(u[VEC_Y], VEC_Y, mot);

		// X (forward)
		u[VEC_X] = sp->Y_throttle;
		check_channel_saturation(VEC_X, mot, &min, &max);
		if(max>MAX_Y_COMPONENT)  max =  MAX_Y_COMPONENT;
		if(min<-MAX_Y_COMPONENT) min = -MAX_Y_COMPONENT;
		rc_saturate_float(&u[VEC_X], min, max);
		// add mixed components to the motors
		add_mixed_input(u[VEC_X], VEC_Y, mot);
	}
	else{
		u[VEC_Y] = 0.0;
		u[VEC_X] = 0.0;
	}

	/***************************************************************************
	* Send ESC motor signals immediately at the end of the control loop
	***************************************************************************/
	for(i=0;i<set->num_rotors;i++){
		// write motor signals to cstate before final saturation so errors
		// may show up in the logs
		cs->m[i] = new_mot[i];
		// Final saturation before sending motor signals to prevent errors
		rc_saturate_float(&new_mot[i], 0.0, 1.0);
		rc_send_esc_pulse_normalized(i+1,new_mot[i]);
	}

	/***************************************************************************
	* Final cleanup, timing, and indexing
	***************************************************************************/
	// Load control inputs into cstate for viewing by outside threads
	for(i=0;i<6;i++) cs->u[i]=u[i];
	// keep track of loops since arming
	cs->loop_index++;
	// log us since arming, mostly for the log
	cs->last_step_us = (rc_nanos_since_epoch()/1000)-start_time_us;

	/***************************************************************************
	* Add new log entry
	***************************************************************************/
	if(set->enable_logging){
		new_log.loop_index	= cs->loop_index;
		new_log.last_step_us	= cs->last_step_us;
		new_log.altitude	= cs->altitude;
		new_log.roll		= cs->roll;
		new_log.pitch		= cs->pitch;
		new_log.yaw		= cs->yaw;
		new_log.v_batt		= cs->v_batt;
		new_log.u_X		= u[VEC_Y];
		new_log.u_Y		= u[VEC_X];
		new_log.u_Z		= u[VEC_Z];
		new_log.u_roll		= u[VEC_ROLL];
		new_log.u_pitch		= u[VEC_PITCH];
		new_log.u_yaw		= u[VEC_YAW];
		new_log.mot_1		= cs->m[0];
		new_log.mot_2		= cs->m[1];
		new_log.mot_3		= cs->m[2];
		new_log.mot_4		= cs->m[3];
		new_log.mot_5		= cs->m[4];
		new_log.mot_6		= cs->m[5];
		add_log_entry(new_log);
	}

	return;
}
