/*
 * mavlink_bridge.h
 *
 *  Created on: Sep 10, 2013
 *      Author: Alan
 */

#ifndef EDUCIAACOPTER_APP_MAVLINK_BRIDGE_H_
#define EDUCIAACOPTER_APP_MAVLINK_BRIDGE_H_

/* MAVLink adapter header */
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "mavlink_types.h"
#include "qUART.h"
#include "FreeRTOS.h"

#define SENSOR_GYRO	(1<<0)
#define SENSOR_ACC	(1<<1)
#define SENSOR_MAG	(1<<2)
#define SENSOR_ABS_PRESSURE	(1<<3)
#define SENSOR_DIFF_PRESSURE (1<<4)
#define SENSOR_GPS	(1<<5)
#define SENSOR_OPTICAL_FLOW	(1<<6)
#define SENSOR_COMPUTER_VISION_POSITION	(1<<7)
#define SENSOR_LASER_POSITION	(1<<8)
#define SENSOR_VICON	(1<<9)

#if 0
void mvalink_send_telemetry(float absolute_pressure, float differential_pressure, float temperature){
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_scaled_pressure_pack(mavlink_system.sysid,mavlink_system.compid,&msg,xTaskGetTickCount(),absolute_pressure,differential_pressure,temperature);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	(*sender_function)(UART_GROUNDCOMM,buf,len);
}

void mavlink_send_attitude(float roll, float pitch, float yaw, float rolld, float pitchd, float yawd){
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_attitude_pack(mavlink_system.sysid,mavlink_system.compid,&msg,xTaskGetTickCount(),roll, pitch, yaw, rolld, pitchd, yawd);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	(*sender_function)(UART_GROUNDCOMM,buf,len);
}

void mavlink_send_hud(float altitude){
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_vfr_hud_pack(mavlink_system.sysid,mavlink_system.compid,&msg,25.5,100.0,250,0,altitude*1000,0.0);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	(*sender_function)(UART_GROUNDCOMM,buf,len);
}

#endif

#endif /* EDUCIAACOPTER_APP_MAVLINK_BRIDGE_H_ */
