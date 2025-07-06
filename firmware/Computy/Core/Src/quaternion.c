/*
 * quaternion.c
 *
 *  Created on: Jul 7, 2025
 *      Author: gregor
 */


#include "quaternion.h"

struct Quaternion quaternion_multiply(struct Quaternion *q1, struct Quaternion *q2) {

	struct Quaternion q3;

	q3.w = q1->w * q2->w - q1->x*q2->x - q1->y * q2->y - q1->z * q2->z;
	q3.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
	q3.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
	q3.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
	return q3;

}
