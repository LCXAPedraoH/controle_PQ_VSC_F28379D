/* =================================================================================
File name:       PI.H 
===================================================================================*/


#ifndef __PI_H__
#define __PI_H__

typedef struct {
    float  Ref;   			// Input: reference set-point
	float  Fbk;   			// Input: feedback
	float  Err;
	float  Out;   			// Output: controller output
	float  Kp;				// Parameter: proportional loop gain
	float  Ki;			    // Parameter: integral gain
	float  Umax;			// Parameter: upper saturation limit
	float  Umin;			// Parameter: lower saturation limit
	float  up;				// Data: proportional term
	float  ui;				// Data: integral term
	float  v1;				// Data: pre-saturated controller output
	float  i1;				// Data: integrator storage: ui(k-1)
	float  w1;				// Data: saturation record: [u(k-1) - v(k-1)]
	void  (*calc)()
	void  (*reset)()
				} PI_CONTROLLER;

typedef PI_CONTROLLER *PI_CONTROLLER_handle;

/*-----------------------------------------------------------------------------
Default initalisation values for the PI_GRANDO objects
-----------------------------------------------------------------------------*/                     
void pi_calc(PI_CONTROLLER_handle);
void pi_reset(PI_CONTROLLER *v);


#define PI_CONTROLLER_DEFAULTS { 0, \
                                 0, \
                                 0, \
                                 0, \
                                 1.0,\
                                 0.0,\
                                 1.0,\
                                 -1.0,\
                                 0.0,\
                                 0.0,\
                                 0.0,\
                                 0.0,\
                                 1.0,\
                                (void (*)(Uint32))pi_calc,\
                                (void (*)(Uint32))pi_reset}

/*------------------------------------------------------------------------------
	PI_GRANDO Macro Definition
------------------------------------------------------------------------------*/

#define PI_MACRO(v)												\
																\
	/* proportional term */ 									\
	v.up = _IQmpy(v.Kp, (v.Ref - v.Fbk));						\
																\
	/* integral term */ 										\
	v.ui = (v.Out == v.v1)?(_IQmpy(v.Ki, v.up)+ v.i1) : v.i1;	\
	v.i1 = v.ui;												\
																\
	/* control output */ 										\
	v.v1 = v.up + v.ui;											\
	v.Out= _IQsat(v.v1, v.Umax, v.Umin);						\
	//v.w1 = (v.Out == v.v1) ? _IQ(1.0) : _IQ(0.0);				\

// ***********************************************************************************
//   This macro works with angles as inputs, hence error is rolled within -pi to +pi
// ***********************************************************************************

#endif // __PI_H__

