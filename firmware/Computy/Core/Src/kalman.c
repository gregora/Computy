/*
 * kalman.c
 *
 *  Created on: Sep 19, 2025
 *      Author: gregor
 */

#include "kalman.h"

void kalman_init(){
    arm_mat_init_f32(&P, 6, 6, P_pdata);
    arm_mat_init_f32(&Q, 6, 6, Q_pdata);
}

