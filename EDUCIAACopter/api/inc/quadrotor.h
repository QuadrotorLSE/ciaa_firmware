/*
 * quadrotor.h
 *
 *  Created on: 11/01/2013
 *      Author: alan
 */

#ifndef QUADROTOR_H_
#define QUADROTOR_H_

#include "joystick.h"
#include "qPIDs.h"
//#include "qPTOS.h"
#include "mavlink.h"

#define K_Z			(700.0)
#define K_THETA		(250.0)
#define K_PHI		(250.0)
#define K_PSI		(300.0)

typedef enum {NAV_ACRO, NAV_ATTI, NAV_ALTHOLD} nav_mode_t;

typedef enum {ESC_STANDBY=0, ESC_ARMED} mode_t;

typedef struct {
	float rate[3];
	float attitude[3]; // in deg
	float setpoint[4];
	float rateCtrlOutput[3];
	float attiCtrlOutput[3];
	uint16_t motorOutput[4];
	uint32_t time;
	float altitude;
	float altitudeCtrlOutput;
	float floor_pressure;
	float current_pressure;
	float temperature;
	float accel[3];
	float quaternion[4];
} SV_t;

typedef struct {
	float gyroBias[3];
} settings_t;

typedef struct {
	float						term[3];
	SV_t 						sv;
	settings_t 					settings;
	joystick_t 					joystick;
	qPID 						rateController[3];
	qPID 						attiController[3];
	qPID 						altitudeController;
//	qPTOS 						data_ptos;
	mavlink_system_t 			mavlink_system;
	mavlink_manual_control_t 	mavlink_control;
	uint16_t 					sysload;
	mode_t 						mode;
}quadrotor_t;

extern quadrotor_t quadrotor;


#endif /* QUADROTOR_H_ */
