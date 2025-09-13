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

struct Quaternion quaternion_conjugate(struct Quaternion *q);

struct Quaternion quaternion_transform_vector(struct Quaternion *q, float v[3]);

#endif /* INC_QUATERNION_H_ */
