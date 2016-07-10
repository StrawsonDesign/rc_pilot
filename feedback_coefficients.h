/*******************************************************************************
* fly_feedback_coefficients.h
*
* Feedback control gains and saturation limits
*******************************************************************************/


// Altitude Controller
#define D0_GAIN					1.0	
#define D0_ORDER				2
#define D0_NUM					{-6.289, 11.910, -5.634 }
#define D0_DEN					{ 1.000, -1.702,  0.702 }
#define MIN_THRUST_COMPONENT	0.15	
#define MAX_THRUST_COMPONENT 	0.85
#define ALT_BOUND_D				0.5 // meters below current altitude
#define ALT_BOUND_U				1.0	// meters above current altitude

// Roll Controller
#define D1_GAIN					1.0	
#define D1_ORDER				2
#define D1_NUM					{-6.289, 11.910, -5.634 }
#define D1_DEN					{ 1.000, -1.702,  0.702 }
#define MAX_ROLL_COMPONENT		0.8 

// Pitch controller
#define D2_GAIN					1.0
#define	D2_ORDER				2
#define D2_NUM					{-6.289, 11.910, -5.634 }
#define D2_DEN					{ 1.000, -1.702,  0.702 }
#define MAX_PITCH_COMPONENT		0.8

// Yaw controller
#define D3_GAIN					1.0
#define	D3_ORDER				2
#define D3_NUM					{-6.289, 11.910, -5.634 }
#define D3_DEN					{ 1.000, -1.702,  0.702 }
#define MAX_YAW_COMPONENT		0.7 
#define YAW_SP_BOUND			0.5

// X,Y limits
#define MAX_X_COMPONENT			1.0
#define MAX_Y_COMPONENT			1.0
