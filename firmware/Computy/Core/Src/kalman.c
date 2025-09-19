/*
 * kalman.c
 *
 *  Created on: Sep 19, 2025
 *      Author: gregor
 */

#include "kalman.h"

float longitude0 = 0.0f; // initial longitude, used for calculating northing
float latitude0 = 0.0f;  // initial latitude, used for calculating easting

arm_matrix_instance_f32 P;
float P_pdata[36];

arm_matrix_instance_f32 Q;
float Q_pdata[36];

void kalman_init(){
    arm_mat_init_f32(&P, 6, 6, P_pdata);
    arm_mat_init_f32(&Q, 6, 6, Q_pdata);
}
