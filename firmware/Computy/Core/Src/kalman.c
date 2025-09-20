/*
 * kalman.c
 *
 *  Created on: Sep 19, 2025
 *      Author: gregor
 */

#include "kalman.h"

float longitude0 = 0.0f; // initial longitude, used for calculating northing
float latitude0 = 0.0f;  // initial latitude, used for calculating easting

arm_matrix_instance_f32 x; // state vector: vx, vy, vz, deltax, deltay, height
float x_pdata[6*1] = {0,0,0,0,0,0};


// PREDICTION
arm_matrix_instance_f32 F;
float F_pdata[6*6];

arm_matrix_instance_f32 F_T;
float F_T_pdata[6*6];

arm_matrix_instance_f32 F_times_P;
float F_times_P_pdata[6*6];

arm_matrix_instance_f32 P;
float P_pdata[6*6] = {1,0,0,0,0,0,
					 0,1,0,0,0,0,
					 0,0,1,0,0,0,
					 0,0,0,1,0,0,
					 0,0,0,0,1,0,
					 0,0,0,0,0,1};

// UPDATE
arm_matrix_instance_f32 y; // measurement error: ex, ey, ez
float y_pdata[3*1] = {0,0,0};


arm_matrix_instance_f32 H;
float H_pdata[3*6] = {
		0,0,0,1,0,0,
		0,0,0,0,1,0,
		0,0,0,0,0,1
};

arm_matrix_instance_f32 H_T;
float H_T_pdata[6*3] = {
		0,0,0,
		0,0,0,
		0,0,0,
		1,0,0,
		0,1,0,
		0,0,1
};

arm_matrix_instance_f32 H_times_P;
float H_times_P_pdata[3*6];

arm_matrix_instance_f32 S;
float S_pdata[3*3];

arm_matrix_instance_f32 S_inv;
float S_inv_pdata[3*3];

arm_matrix_instance_f32 P_times_H_T;
float P_times_H_T_pdata[6*3];

arm_matrix_instance_f32 K;
float K_pdata[6*3];

arm_matrix_instance_f32 K_times_y;
float K_times_y_pdata[6*1];

arm_matrix_instance_f32 K_times_H;
float K_times_H_pdata[6*6];

arm_matrix_instance_f32 P_new;
float P_new_pdata[6*6];


void kalman_init(){
    arm_mat_init_f32(&x, 6, 1, x_pdata);
	arm_mat_init_f32(&F, 6, 6, F_pdata);
    arm_mat_init_f32(&F_T, 6, 6, F_T_pdata);
    arm_mat_init_f32(&F_times_P, 6, 6, F_times_P_pdata);
    arm_mat_init_f32(&P, 6, 6, P_pdata);


    arm_mat_init_f32(&y, 3, 1, y_pdata);
    arm_mat_init_f32(&H, 3, 6, H_pdata);
    arm_mat_init_f32(&H_T, 6, 3, H_T_pdata);
    arm_mat_init_f32(&H_times_P, 3, 6, H_times_P_pdata);
    arm_mat_init_f32(&S, 3, 3, S_pdata);
    arm_mat_init_f32(&S_inv, 3, 3, S_inv_pdata);
    arm_mat_init_f32(&P_times_H_T, 6, 3, P_times_H_T_pdata);
    arm_mat_init_f32(&K, 6, 3, K_pdata);
    arm_mat_init_f32(&K_times_y, 6, 1, K_times_y_pdata);
    arm_mat_init_f32(&K_times_H, 6, 6, K_times_H_pdata);
    arm_mat_init_f32(&P_new, 6, 6, P_new_pdata);

}

void kalman_predict(float dt, float ax, float ay, float az) {

	if(latitude0 == 0 && longitude0 == 0){
		// Ignore prediction update if initial gps position was established
		return;
	}

	// predict new values
	x_pdata[0] += dt*ax;
	x_pdata[1] += dt*ay;
	x_pdata[2] += dt*az;

	x_pdata[3] += x_pdata[0]*dt;
	x_pdata[4] += x_pdata[1]*dt;
	x_pdata[5] += x_pdata[2]*dt;

	// calculate covariance matrix
	for(int i = 0; i < 36; i++){
		F_pdata[i] = 0;
	}
	F_pdata[0] = 1;
	F_pdata[7] = 1;
	F_pdata[14] = 1;
	F_pdata[21] = 1;
	F_pdata[28] = 1;
	F_pdata[35] = 1;

	F_pdata[18] = dt;
	F_pdata[25] = dt;
	F_pdata[32] = dt;

	arm_mat_trans_f32(&F, &F_T);

	arm_mat_mult_f32(&F, &P, &F_times_P);
	arm_mat_mult_f32(&F_times_P, &F_T, &P);

	P_pdata[0] += 0.1 * dt;
	P_pdata[7] += 0.1 * dt;
	P_pdata[14] += 0.1 * dt;

}

void kalman_update(float latitude, float longitude, float height){
	if(latitude0 == 0.0f && longitude0 == 0.0f){
		latitude0 = latitude;
		longitude0 = longitude;
		x_pdata[5] = -height;
	}

	float deltax = (latitude - latitude0) / 360.0f * EARTH_CIRCUMFERENCE;
	float deltay = (longitude - longitude0) / 360.0f * EARTH_CIRCUMFERENCE * cos(latitude * 3.1415 / 180.0);

	y_pdata[0] =  deltax - x_pdata[3];
	y_pdata[1] =  deltay - x_pdata[4];
	y_pdata[2] = -height - x_pdata[5];

	arm_mat_mult_f32(&H, &P, &H_times_P);
	arm_mat_mult_f32(&H_times_P, &H_T, &S);

	S_pdata[0] += 5.0f; // 5m lateral error
	S_pdata[4] += 5.0f; // 5m lateral error
	S_pdata[8] += 20.0f; // 20m height error

	// S is modified by the inverse!
	arm_mat_inverse_f32(&S, &S_inv);

	arm_mat_mult_f32(&P, &H_T, &P_times_H_T);
	arm_mat_mult_f32(&P_times_H_T, &S_inv, &K); // Near-optimal Kalman gain

	arm_mat_mult_f32(&K, &y, &K_times_y);

	arm_mat_add_f32(&x, &K_times_y, &x); // Update state estimate

	arm_mat_mult_f32(&K, &H, &K_times_H);

	for(int i = 0; i < 6; i++){
		for(int j = 0; j < 6; j++){
			K_times_H_pdata[6*i + j] *= -1;

			if(i == j){
				K_times_H_pdata[6*i + j] += 1;
			}
		}
	}

	arm_mat_mult_f32(&K_times_H, &P, &P_new);

	memcpy(P_pdata, P_new_pdata, sizeof(float)*36);

}


void kalman_result(float* latitude_ptr, float* longitude_ptr, float* height_ptr) {
	*latitude_ptr = latitude0 + x_pdata[3] * 360.0f / EARTH_CIRCUMFERENCE;
	*longitude_ptr = longitude0 + x_pdata[4] * 360.0f / EARTH_CIRCUMFERENCE / cos(latitude0 * 3.1415 / 180.0);
	*height_ptr = -x_pdata[5];
}

