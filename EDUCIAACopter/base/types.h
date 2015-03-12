/*
 * types.h
 *
 *  Created on: 26/10/2012
 *      Author: alan
 */

#ifndef EDUCIAACOPTER_BASE_TYPES_H_
#define EDUCIAACOPTER_BASE_TYPES_H_

#include "lpc_types.h"
#include <stdint.h>
//typedef Bool bool;
typedef enum{
	RET_OK=0,
	RET_ERROR,
	RET_ALREADY_INIT,
	RET_MSG_OK,
	RET_MSG_ERROR,
	RET_MSG_BYTES_REMAINING
}ret_t;

typedef enum{
	DEVICE_NOT_READY = 0,
	DEVICE_READY
}status_t;

#ifndef __SIZE_T_DEFINED
#define __SIZE_T_DEFINED
#if __SIZEOF_INT < __SIZEOF_VOID_P
typedef unsigned long size_t;
#else
typedef unsigned int size_t;
#endif
#endif

#define PI 3.14159265359
#define M_PI 3.14159265358979323846
#define FLT_EPSILON 1.19209290E-07F

typedef struct{
	int16_t raw_gyro[3];		// Last raw measure of the gyros
	int16_t raw_accel[3];		// Last raw measure of the accelerometers
	float scale_gyro;			// Gyro scale in LSB/deg/sec
	float scale_accel;			// Accel scale in LSB/g

	float quat[4];				// Calculated quaternion
	float angular_velocity[3];  // Scaled angular velocity in deg/sec
	float acceleration[3];		// Scaled acceleration in m/sec
	float attitude[3]; 			// Euler angles in roll, pitch, yaw format in deg
}IMU_t;




#endif /* EDUCIAACOPTER_BASE_TYPES_H_ */
