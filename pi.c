/*
 * pi.c
 *
 *  Created on: 24 de mai de 2023
 *      Author: Pedro Henrique
 */
#include "pi.h"

void pi_calc(PI_CONTROLLER *v){
    /* proportional term */
    v->up = v->Kp*v->Err;
    /* integral term */
    if (v->Out == v->v1) {
        v->ui = v->Ki * v->up + v->i1;
    } else {
        v->ui = v->i1;
    }

    /* control output */
    v->v1 = v->up + v->ui;

    if (v->v1 > v->Umax)
        v->Out =  v->Umax;
    else if (v->v1 < v->Umin)
        v->Out =  v->Umin;
    else
        v->Out = v->v1;

    //v.w1 = (v.Out == v.v1) ? _IQ(1.0) : _IQ(0.0);
}

void pid_reg3_reset(PIDREG3 *v){
    v->Err = 0;
    v->Fdb = 0;
    v->Out = 0;
    v->v1 = 0;
    v->Ui = 0;
    v->Up1 = 0;

    v->SatErr = 0;
    v->Out = 0;
}
