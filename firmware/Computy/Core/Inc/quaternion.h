/*
 * quaternion.h
 *
 *  Created on: Jul 7, 2025
 *      Author: gregor
 */

#ifndef INC_QUATERNION_H_
#define INC_QUATERNION_H_


struct Quaternion {
	float w;
	float x;
	float y;
	float z;
};

struct Quaternion quaternion_multiply(struct Quaternion *q1, struct Quaternion *q2);

#endif /* INC_QUATERNION_H_ */
