/**
 * @file state_estimator.c
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
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/bmp.h>

#include <rc_pilot_defs.h>
#include <settings.h>


#define TWO_PI (M_PI*2.0)

state_estimate_t state_estimate; // extern variable in state_estimator.h


static double dt; // controller timestep
static int num_yaw_spins;
static double last_yaw;
static double tmp;

static rc_mpu_data_t mpu_data;
static rc_bmp_data_t bmp_data;

// filters
static rc_kalman_t alt_kf;
static rc_vector_t u,y;
static rc_filter_t acc_lp;
static rc_filter_t altitude_lp;


// local functions
static double __batt_voltage();
static int __init_altitude_kf()
static int __alt_kf_march();


static double __batt_voltage()
{
	float tmp;

	tmp = rc_adc_dc_jack();
	if(tmp<3.0f) tmp = settings.v_nominal;
	return tmp;
}

static int __alt_kf_march()
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
	if(alt_kf.step==0){
		alt_kf.x_est.d[0] = bmp_data.alt_m;
		rc_filter_prefill_inputs(&acc_lp, accel_vec[2]-9.80665);
		rc_filter_prefill_outputs(&acc_lp, accel_vec[2]-9.80665);
	}

	// calculate acceleration and smooth it just a tad
	rc_filter_march(&acc_lp, accel_vec[2]-9.80665);
	u.d[0] = acc_lp.newest_output;

	// don't bother filtering Barometer, kalman will deal with that
	y.d[0] = bmp_data.alt_m;
	rc_kalman_update_lin(&alt_kf, u, y);

	// altitude estimate
	state_estimate.altitude_bmp = rc_filter_march(&altitude_lp,bmp_data.alt_m);
	state_estimate.altitude_kf = alt_kf.x_est.d[0];
	state_estimate.alt_kf_vel = alt_kf.x_est.d[1];
	state_estimate.alt_kf_accel = alt_kf.x_est.d[2];

	return 0;
}




static int __init_altitude_kf()
{
	//initialize altitude kalman filter and bmp sensor
	rc_matrix_t F = rc_matrix_empty();
	rc_matrix_t G = rc_matrix_empty();
	rc_matrix_t H = rc_matrix_empty();
	rc_matrix_t Q = rc_matrix_empty();
	rc_matrix_t R = rc_matrix_empty();
	rc_matrix_t Pi = rc_matrix_empty();

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
	if(rc_kalman_alloc_lin(&alt_kf,F,G,H,Q,R,Pi)==-1) return -1;
	rc_kalman_free(&F);
	rc_kalman_free(&G);
	rc_kalman_free(&H);
	rc_kalman_free(&Q);
	rc_kalman_free(&R);
	rc_kalman_free(&Pi);

	// initialize the little LP filter to take out accel noise
	if(rc_filter_first_order_lowpass(&acc_lp, dt, 20*dt)) return -1;

	// initialize a LP on baromter for comparison to KF
	if(rc_filter_butterworth_lowpass(&altitude_lp, 2, dt, ALT_CUTOFF_FREQ)) return -1;

	// init barometer and read in first data
	if(rc_bmp_read(&bmp_data)) return -1;
	rc_filter_prefill_inputs(&altitude_lp, bmp_data.alt_m);
	rc_filter_prefill_outputs(&altitude_lp, bmp_data.alt_m);

	return 0;
}

int state_estimator_init()
{
	// make local dt variable since it's used so frequently.
	dt = 1.0/settings.feedback_hz;

	// init altitude kalman filter
	if(__init_altitude_kf()) return -1;

	// init the battery filter low pass filter
	rc_filter_moving_average(&batt_lp, 20, 0.01);
	tmp = __batt_voltage();
	rc_filter_prefill_inputs(&batt_lp, tmp);
	rc_filter_prefill_outputs(&batt_lp, tmp);

	return 0;
}


int state_estimator_cleanup()
{
	rc_filter_free(&batt_lp);
	rc_filter_free(&altitude_lp);
	rc_kalman_free(&alt_kf)
	return 0;
}


int state_estimator_march()
{
	double tmp, yaw_reading;

	if(state_estimate.initialized==0){
		fprintf(stderr, "ERROR in feedback_state_estimate, feedback controller not initialized\n");
		return -1;
	}

	// collect new IMU roll/pitch data
	if(settings.enable_magnetometer){
		state_estimate.roll  = mpu_data.fused_TaitBryan[TB_ROLL_Y];
		state_estimate.pitch = mpu_data.fused_TaitBryan[TB_PITCH_X];
		yaw_reading  = mpu_data.fused_TaitBryan[TB_YAW_Z];

	}
	else{
		state_estimate.roll  = mpu_data.dmp_TaitBryan[TB_ROLL_Y];
		state_estimate.pitch = mpu_data.dmp_TaitBryan[TB_PITCH_X];
		yaw_reading  = mpu_data.dmp_TaitBryan[TB_YAW_Z];
	}

	state_estimate.roll_rate = mpu_data.gyro[0];
	state_estimate.pitch_rate = mpu_data.gyro[1];
	state_estimate.yaw_rate = -mpu_data.gyro[2];

	// yaw is more annoying since we have to detect spins
	// also make sign negative since NED coordinates has Z point down
	tmp = -yaw_reading + (num_yaw_spins * TWO_PI);
	//detect the crossover point at +-PI and write new value to core state
	if(tmp-last_yaw < -M_PI) num_yaw_spins++;
	else if (tmp-last_yaw > M_PI) num_yaw_spins--;
	// finally num_yaw_spins is updated and the new value can be written
	state_estimate.yaw = -yaw_reading + (num_yaw_spins * TWO_PI);
	last_yaw = state_estimate.yaw;

	// filter battery voltage.
	state_estimate.v_batt = rc_filter_march(&batt_lp,__batt_voltage());

	return 0;
}

