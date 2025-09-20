/*
 * kalman.h
 *
 *  Created on: Sep 19, 2025
 *      Author: gregor
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#define EARTH_CIRCUMFERENCE 40075000.0f

#include "arm_math.h"

void kalman_init();
void kalman_predict(float dt, float ax, float ay, float az);
void kalman_update(float latitude, float longitude, float height);
void kalman_result(float* latitude_ptr, float* longitude_ptr, float* height_ptr);
#endif /* INC_KALMAN_H_ */
