/*******************************************************************************
* feedback_controller.c
*
* Here lies the heart and soul of the operation. I wish the whole flight
* controller could be just this file, woe is me. initialize_controllers()
* pulls in the control constants from json_settings and sets up the 
* discrete controllers. From then on out, feedback_controller() should be called
* by the IMU interrupt at feedbacl_hz until the program is shut down. 
* feedback_controller() will monitor the setpoint which is constantly being 
* changed by setpoint_manager(). It also does state estimation to update 
* core_state() even when the controller is disarmed. When controllers are
* enabled or disabled mid-flight by mode switches then the controllers are
* started smoothly.
*******************************************************************************/

#include <stdio.h>
#include <roboticscape.h>
#include "fly_defs.h"
#include "fly_types.h"
#include "fly_function_declarations.h"


arm_state_t arm_state;

// discrete controllers
// altitude, roll, pitch, yaw
d_filter_t D0, D1, D2, D3;

// pointers to outside structs
setpoint_t sp;
cstate_t cs;
imu_data_t* imu;
user_input_t* ui;

// one log entry, passed to log manager if logging enabled
log_entry_t new_log;

int num_yaw_spins;
int last_yaw;
float u[6], mot[8], tmp;
uint64_t loop_index;

// rpy and altitude controllers need setup if being turned on mid flight
// so keep track of last state to detect changes.
int last_en_alt_ctrl;
int last_en_rpy_ctrl;
float last_usr_thr;

/*******************************************************************************
* int disarm_controller()
*	
* This is how outside functions should stop the flight controller.
*******************************************************************************/
int disarm_controller(){
	arm_state = DISARMED;
	set_led(RED,1);
	set_led(GREEN,0); 
	if(ENABLE_LOGGING) stop_log_manager();
	return 0;
}

/*******************************************************************************
* int disarm_controller()
*	
* This is how outside functions should stop the flight controller.
*******************************************************************************/
int arm_controller(){
	zero_out_controller();
	loop_index = 0;
	set_led(RED,0);
	set_led(GREEN,1); 
	zero_out_controllers();
	last_alt_ctrl_en = sp;
	last_usr_thr = MIN_THRUST_COMPONENT;
	if(ENABLE_LOGGING) start_log_manager();
	arm_state = ARMED;
	return 0;
}

/*******************************************************************************
* arm_state_t get_controller_arm_state()
*	
* Returns the arm state of the controller so outside functions, namely the
* setpoint_manager, can tell if the controller is armed or not.
*******************************************************************************/
arm_state_t get_controller_arm_state(){
	return arm_state;
}

/*******************************************************************************
* initialize_controller()
*
* initial setup of all feedback controllers. Should only be called once on
* program start. 
*******************************************************************************/
int initialize_controller(cstate_t* cstate, setpoint_t* setpoint, \
									imu_data_t* imu_data, user_input_t* ui){
	float num, den;

	// make local copies of pointers to global structs
	cs = cstate;
	sp = setpoint;
	imu = imu_data;
	ui = user_input_t;

	// set up altitude controller
	float num = D0_NUM;
	float den = D0_DEN;
	D0 = create_filter(D0_ORDER, DT, num, den);
	D0.gain = D0_GAIN;
	enable_saturation(&D0, MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);

	// set up roll controller
	float num = D1_NUM;
	float den = D1_DEN;
	D1 = create_filter(D1_ORDER, DT, num, den);
	D1.gain = D1_GAIN;
	enable_saturation(&D1, MIN_ROLL_COMPONENT, MAX_ROLL_COMPONENT);

	// set up altitude controller
	float num = D2_NUM;
	float den = D2_DEN;
	D2 = create_filter(D2_ORDER, DT, num, den);
	D2.gain = D2_GAIN;
	enable_saturation(&D2, MIN_PITCH_COMPONENT, MAX_PITCH_COMPONENT);

	// set up altitude controller
	float num = D3_NUM;
	float den = D3_DEN;
	D3 = create_filter(D3_ORDER, DT, num, den);
	D3.gain = D3_GAIN;
	enable_saturation(&D3, MIN_YAW_COMPONENT, MAX_YAW_COMPONENT);

	arm_state = DISARMED;
	return 0;
}

/*******************************************************************************
* zero_out_controller()
*
* clear the controller memory
*******************************************************************************/
int zero_out_controller(){
	reset_filter(&D0);
	reset_filter(&D1);
	reset_filter(&D2);
	reset_filter(&D3);
	return 0;
}

/*******************************************************************************
* feedback_controller()
*	
* Should be called by the IMU interrupt at SAMPLE_RATE_HZ
*******************************************************************************/
int feedback_controller(){
	int i;
	float tmp, min, max;
	float mot[ROTORS];
	
	/***************************************************************************
	*	STATE_ESTIMATION
	*	read sensors and compute the state regardless of if the controller
	*	is ARMED or DISARMED
	***************************************************************************/
	// collect new IMU roll/pitch data
	cs->roll   = imu->fusedTaitBryan[VEC3_Y];
	cs->pitch  = imu->fusedTaitBryan[VEC3_X];
	// current roll/pitch/yaw rates straight from gyro 
	cs->dRoll  = imu->gyro[VEC3_Y];
	cs->dPitch = imu->gyro[VEC3_X];
	cs->dYaw   = imu->gyro[VEC3_Z];
		
	// yaw is a bit more complicated as the imu yaw can jump between +-PI
	// but we need to maintain continuous rotation with num_yaw_spins
	// if this is the first loop since being armed, reset yaw spins
	if(last_arm_state==DISARMED && sp->arm_state!=DISARMED){	
		num_yaw_spins = 0;
		last_yaw = cs->yaw;
	}
	tmp = imu->fusedTaitBryan[VEC3_Z] + (num_yaw_spins * TWO_PI);
	// detect the crossover point at +-PI and write new value to core state
	if(tmp-last_yaw < -PI) num_yaw_spins++;
	else if (tmp-last_yaw > PI) num_yaw_spins--;
	// finally num_yaw_spins is updated and the new value can be written 
	cs->yaw = imu->fusedTaitBryan[VEC3_Z] + (num_yaw_spins * TWO_PI);
	last_yaw = cs->yaw;

	// TODO: altitude estimate
		
	
	/***************************************************************************
	* Now check for all conditions that prevent normal running
	***************************************************************************/
	if(get_state()!=RUNNING){
		disarm_controller();
		set_motors_to_zero();
		last_arm_state = DISARMED;
		return 0;
	}
	
	// If disarmed, keep sending 0 to the escs to keep them awake and quiet,
	// this also ensures they get woken up properly as we will be in this
	// disarmed state for a couple seconds at least while the user gets ready
	// to enter the arming sequence
	if(arm_state==DISARMED){
		set_motors_to_zero();
		last_arm_state = DISARMED;
		return 0;
	}

	// check for a tipover
	if(fabs(cs->roll)>TIP_ANGLE || fabs(cs->pitch)>TIP_ANGLE){
		disarm_controller();
		printf("\n TIPOVER DETECTED \n");
		set_motors_to_zero();
		last_arm_state = DISARMED;
		return 0;
	}
	

	/***************************************************************************
	* If transitioning from direct throttle to altitude control, prefill the 
	* filter wth current throttle input to make smooth transition. This is also
	* true if taking off for the first time in altitude mode as arm_controller 
	* sets up last_alt_ctrl_en and last_usr_thr every time controller arms
	***************************************************************************/
	if(sp->altitude_ctrl_en && !last_alt_ctrl_en){
		sp->altitude = cs->alt; // set altitude setpoint to current altitude
		reset_filter(&D0);
		prefill_filter_outputs(&D0,last_usr_thr);
		last_alt_ctrl_en = 1;
	}
		

	/***************************************************************************
	* We are about to start marching the individual SISI controllers forward.
	* Start by zeroing out the motors signals then add from there.
	***************************************************************************/
	for(i=0;i<ROTORS;i++) mot[i] = 0.0;

	/***************************************************************************
	* Throttle/Altitude Controller
	***************************************************************************/
	// if altitude control in not enabled, apply direct throttle
	if(!sp->altitude_ctrl_en){
		// save throttle in case of transition to altitude control
		last_usr_thr = sp->Z_throttle; 
		tmp = sp->Z_throttle / (cos(cs->roll)*cos(cs->pitch));
		saturate_float(&tmp, MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);
		u[VEC_THR] = tmp;
		add_mixed_input(u[VEC_THR], VEC_THR, mot);
		last_alt_ctrl_en = 0;
	}
	// altitude controller
	else{
		sp->altitude += sp->altitude_rate*DT;
		saturate_float(&sp->altitude, cs->alt-ALT_BOUND_D, cs->alt+ALT_BOUND_U);
		D0.gain = D0_GAIN * V_NOMINAL/cs->vbatt;
		tmp = march_filter(&D0, sp->altitude-cs->alt);
		u[VEC_THR] = tmp / cos(cs->roll)*cos(cs->pitch);
		saturate_float(&u[VEC_THR], MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);
		add_mixed_input(u[VEC_THR], VEC_THR, mot);
	}

	/***************************************************************************
	* Roll Controller          
	***************************************************************************/
	check_channel_saturation(VEC_ROLL, mot, &min, &max);
	if(max>MAX_ROLL_COMPONENT)  max =  MAX_ROLL_COMPONENT;
	if(min<-MAX_ROLL_COMPONENT) min = -MAX_ROLL_COMPONENT;
	enable_saturation(&D1, min, max);
	D1.gain = D1_GAIN * V_NOMINAL/cs->vbatt;
	u[VEC_ROLL]=march_filter(&D1, sp->roll - cs->roll);
	add_mixed_input(u[VEC_ROLL], VEC_ROLL, mot);

	/***************************************************************************
	* Pitch Controller           
	***************************************************************************/
	check_channel_saturation(VEC_PITCH, mot, &min, &max);
	if(max>MAX_PITCH_COMPONENT)  max =  MAX_PITCH_COMPONENT;
	if(min<-MAX_PITCH_COMPONENT) min = -MAX_PITCH_COMPONENT;
	enable_saturation(&D2, min, max);
	D2.gain = D2_GAIN * V_NOMINAL/cs->vbatt;
	u[VEC_PITCH] = march_filter(&D2, sp->pitch - cs->pitch);	
	add_mixed_input(u[VEC_PITCH], VEC_PITCH, mot);	
	
	/***************************************************************************
	* Yaw Controller
	***************************************************************************/
	// if throttle stick is down (waiting to take off) keep yaw setpoint at
	// current heading, otherwide update by yaw rate
	if(ui->throttle_stick < -0.95) sp->yaw = cs->yaw;
	else sp->yaw += DT*sp->yaw_rate;
	saturate_float(&sp->yaw, cs->yaw-YAW_SP_BOUND, cs->yaw+YAW_SP_BOUND);
	check_channel_saturation(VEC_YAW, mot, &min, &max);
	if(max>MAX_YAW_COMPONENT)  max =  MAX_YAW_COMPONENT;
	if(min<-MAX_YAW_COMPONENT) min = -MAX_YAW_COMPONENT;
	enable_saturation(&D3, min, max);
	D3.gain = D3_GAIN * V_NOMINAL/cs->vbatt;
	u[VEC_YAW] = march_filter(&D3, sp->yaw - cs->yaw);
	add_mixed_input(u[VEC_YAW], VEC_YAW, mot);
	

	/***************************************************************************
	* Side and Forward inputs, only when 6dof is enabled
	***************************************************************************/
	if(sp->6dof_en){
		// check for saturation limits and bound that in the hard limit
		u[VEC_SIDE] = sp->X_throttle;
		check_channel_saturation(VEC_SIDE, mot, &min, &max);
		if(max>MAX_X_COMPONENT)  max =  MAX_X_COMPONENT;
		if(min<-MAX_X_COMPONENT) min = -MAX_X_COMPONENT;
		saturate_float(&u[VEC_SIDE], min, max);
		// add mixed components to the motors
		add_mixed_input(u[VEC_SIDE], VEC_SIDE, mot);

		// check for saturation limits and bound that in the hard limit
		u[VEC_FWD] = sp->Y_throttle;
		check_channel_saturation(VEC_FWD, mot, &min, &max);
		if(max>MAX_Y_COMPONENT)  max =  MAX_Y_COMPONENT;
		if(min<-MAX_Y_COMPONENT) min = -MAX_Y_COMPONENT;
		saturate_float(&u[VEC_FWD], min, max);
		// add mixed components to the motors
		add_mixed_input(u[VEC_FWD], VEC_SIDE, mot);
	}
	else{
		u[VEC_SIDE] = 0.0;
		u[VEC_FWD]  = 0.0;
	}


	/***************************************************************************
	* Send ESC motor signals immediately at the end of the control loop
	***************************************************************************/
	for(i=0;i<ROTORS;i++){
		saturate_float(&new_mot[i], 0.0, 1.0);
		send_esc_pulse_normalized(i+1,new_mot[i]);
		cs->mot[i] = new_mot[i];	
	}
	
	/***************************************************************************
	* Add new log entry
	***************************************************************************/
	if(ENABLE_LOGGING){
		new_log.loop_index 	= loop_index;
		new_log.alt 	 	= cs->alt;
		new_log.roll 	 	= cs->roll;
		new_log.pitch 	 	= cs->pitch;
		new_log.yaw 	 	= cs->yaw;
		new_log.u_thr		= u[VEC_THR];
		new_log.u_roll		= u[VEC_ROLL];
		new_log.u_pitch		= u[VEC_PITCH];
		new_log.u_yaw		= u[VEC_YAW];
		new_log.u_X			= u[VEC_SIDE];
		new_log.u_Y			= u[VEC_FWD];
		new_log.mot_1		= mot[0];
		new_log.mot_2		= mot[1];
		new_log.mot_3		= mot[2];
		new_log.mot_4		= mot[3];
		new_log.mot_5		= mot[4];
		new_log.mot_6		= mot[5];
		new_log.vbatt		= cs->vbatt;
		add_log_entry(new_log);
	}

	loop_index++;
	return 0;
}