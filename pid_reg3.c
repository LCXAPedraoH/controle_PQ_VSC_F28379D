/*=====================================================================================
 File name:        PID_REG3.C  (IQ version)                  
                    
 Originator:	Digital Control Systems Group
			Texas Instruments

 Description:  The PID controller with anti-windup                   

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
-------------------------------------------------------------------------------------*/
/*#include "pid_reg3.h"

void pid_reg3_calc(PIDREG3 *v)
{	
    // Compute the error
    //v->Err = v->Ref - v->Fdb;
    // Compute the proportional output
    v->Up = v->Kp*v->Err;
    // Compute the integral output
    v->Ui = v->Ui + v->Ki*(v->Up+v->Up1)*200 + (v->Kc*v->SatErr);
    // Compute the derivative output
    //v->Ud = (v->Kd*(v->Up - v->Up1));
    // Compute the pre-saturated output
    v->OutPreSat = v->Up + v->Ui;
    // Saturate the output
    if (v->OutPreSat > v->OutMax)
      v->Out =  v->OutMax;
    else if (v->OutPreSat < v->OutMin)
      v->Out =  v->OutMin;
    else
      v->Out = v->OutPreSat;

    // Compute the saturate difference
    v->SatErr = v->Out - v->OutPreSat;

    // Update the previous proportional output
    v->Up1 = v->Up;
}

void pid_reg3_reset(PIDREG3 *v){
	v->Err = 0;
	v->Fdb = 0;
	v->Out = 0;
	v->OutPreSat = 0;
	v->Ui = 0;
	v->Up1 = 0;
	v->Ud = 0;
	v->SatErr = 0;
	v->Out = 0;
}
*/





/*
 * calc_functions.c
 *
 *  Created on: 22 May 2020
 *      Author: Almir Braggio
 */

#include "PID_REG3.H"

//
// Defines
//

//
// Globals
//

/*#if 0
// AC stage: PLL voltage reference
pi_param_t g_pi_vsi_pll;

// AC stage: on-grid mode
pi_param_t g_pi_vsi_ongrid_c_d;     // current "d"
pi_param_t g_pi_vsi_ongrid_c_q;     // current "q"

// AC stage: off-grid mode
pi_param_t g_pi_vsi_offgrid_v_d;    // voltage "d"
pi_param_t g_pi_vsi_offgrid_v_q;    // voltage "q"
pi_param_t g_pi_vsi_offgrid_c_d;    // current "d"
pi_param_t g_pi_vsi_offgrid_c_q;    // current "q"
#endif
*/




//
// Proportionalintegral
//
void  pid_reg3_calc(pi_param_t * control)
{
    float pi_pos = 0.0;
    float pi_neg = 0.0;
    float pi_err;
    float pi_prop;

    pi_err = control->reference - control->measure;
    pi_prop = pi_err * control->kp;


    // Initial checking
    if (pi_prop >= control->max)
    {
        pi_prop = control->max;
    }
    if(pi_prop <= control->min)
    {
        pi_prop = control->min;
    }


    // Pro-initial checking
    if (pi_prop >= 0.0)
    {
        pi_pos = control->max - pi_prop;
        pi_neg =  -400;
    }
    else
    {
        pi_neg = control->min - pi_prop;
        pi_pos = 400;
    }

    control->integral_result = control->integral_result + \
            (pi_err * control->ki /20000);

    if (control->integral_result >= pi_pos)
    {
        control->integral_result = pi_pos;
    }
    if (control->integral_result <= pi_neg)
    {
        control->integral_result = pi_neg;
    }


    control->out = (control->integral_result + pi_prop);

}




