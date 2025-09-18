#ifndef INC_IBUS_H_
#define INC_IBUS_H_

#include "arm_math.h"

float longitude0 = 0.0f; // initial longitude, used for calculating northing
float latitude0 = 0.0f;  // initial latitude, used for calculating easting

arm_matrix_instance_f32 P;
float P_pdata[36];

arm_matrix_instance_f32 Q;
float Q_pdata[36];

void kalman_init();

#endif
