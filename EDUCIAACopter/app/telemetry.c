/*
 * telemetry.c
 *
 *  Created on: 13/09/2013
 *      Author: alan
 */

#include "chip.h"
#include "board.h"
#include "quadrotor.h"
#include "qboard.h"
#include "DebugConsole.h"
#include "types.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "mavlink.h"
#include "mavlink_bridge.h"
#include "qUART.h"

//#define PI 3.14159265359
extern float current_alt_sp;
extern IMU_t mpu;

uint16_t quadrotor_msg_attitude 			( mavlink_message_t * msg, uint8_t * buf );
uint16_t quadrotor_msg_vfr_hud 				( mavlink_message_t * msg, uint8_t * buf );
uint16_t quadrotor_msg_manual_control 		( mavlink_message_t * msg, uint8_t * buf );
uint16_t quadrotor_msg_attitude_quaternion 	( mavlink_message_t * msg, uint8_t * buf );


void Telemetry ( void * p )
{
	mavlink_message_t 	msg;
	uint8_t 			buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t 			len;

	debug("\r\nInicializando Telemetry\r\n");
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount ();
	debug("\r\Telemetry [OK]\r\n");

	while ( 1 )
	{
		if ( qUART_Status(UART_GROUNDCOMM) == DEVICE_READY )
		{
			#if ( TELEMETRIA_ENVIO__ESTADO == ESTADO__ON )
				#if ( TELEMETRIA_ENVIO__attitude == ESTADO__ON )
					len = quadrotor_msg_attitude( &msg, buf );
					qUART_Send( UART_GROUNDCOMM, buf, len );
				#endif
				#if ( TELEMETRIA_ENVIO__vfr == ESTADO__ON )
					len = quadrotor_msg_vfr_hud( &msg, buf );
					qUART_Send( UART_GROUNDCOMM, buf, len );
				#endif
				#if ( TELEMETRIA_ENVIO__manual_control == ESTADO__ON )
					len = quadrotor_msg_manual_control( &msg, buf );
					qUART_Send( UART_GROUNDCOMM, buf, len );
				#endif
				#if ( TELEMETRIA_ENVIO__quaternion == ESTADO__ON )
					len = quadrotor_msg_attitude_quaternion( &msg, buf );
					qUART_Send( UART_GROUNDCOMM, buf, len );
				#endif
			#endif
		}
		vTaskDelayUntil( &xLastWakeTime, 50 / portTICK_RATE_MS );
	}
}

inline uint16_t quadrotor_msg_attitude ( mavlink_message_t * msg, uint8_t * buf )
{
	mavlink_msg_attitude_pack(	quadrotor.mavlink_system.sysid,
								quadrotor.mavlink_system.compid,
								msg,
								quadrotor.sv.time,
								quadrotor.sv.attitude[ROLL]*PI/180.0,
								quadrotor.sv.attitude[PITCH]*PI/180.0,
								quadrotor.sv.attitude[YAW]*PI/180.0,
								quadrotor.sv.rate[ROLL],
								quadrotor.sv.rate[PITCH],
								quadrotor.sv.rate[YAW]);
	return mavlink_msg_to_send_buffer( buf, msg );
}

inline uint16_t quadrotor_msg_vfr_hud ( mavlink_message_t * msg, uint8_t * buf )
{
	mavlink_msg_vfr_hud_pack(	quadrotor.mavlink_system.sysid,
								quadrotor.mavlink_system.compid,
								msg,
								quadrotor.rateController[ROLL].K, // Airspeed
								quadrotor.rateController[ROLL].Ti, // Groundspeed
								250, //TODO: Change to realheading
								quadrotor.sv.altitudeCtrlOutput*100,
								quadrotor.sv.altitude,
								0.0);  //TODO: Change to ascent rate
	return mavlink_msg_to_send_buffer( buf, msg );

//			mavlink_msg_vfr_hud_pack(quadrotor.mavlink_system.sysid,
//									quadrotor.mavlink_system.compid,
//									&msg,
//									quadrotor.sv.setpoint[ALTITUDE], // Airspeed
//									quadrotor.sv.setpoint[ALTITUDE], // Groundspeed
//									250, //TODO: Change to realheading
//									quadrotor.sv.altitudeCtrlOutput*100,
//									quadrotor.sv.altitude,
//									quadrotor.sv.setpoint[ALTITUDE]  //TODO: Change to ascent rate
//									);
//			len = mavlink_msg_to_send_buffer(buf, &msg);
//			qUART_Send(UART_GROUNDCOMM,buf,len);
}

inline uint16_t quadrotor_msg_manual_control ( mavlink_message_t * msg, uint8_t * buf )
{
	mavlink_msg_manual_control_pack(	quadrotor.mavlink_system.sysid,
										quadrotor.mavlink_system.compid,
										msg,
										quadrotor.mavlink_system.sysid,
										quadrotor.sv.motorOutput[MOTOR1_ID], //quadrotor.mavlink_control.x,
										quadrotor.sv.motorOutput[MOTOR2_ID], //quadrotor.mavlink_control.y,
										quadrotor.sv.motorOutput[MOTOR3_ID], //quadrotor.mavlink_control.z,
										quadrotor.sv.motorOutput[MOTOR4_ID], //quadrotor.mavlink_control.r,
										quadrotor.mavlink_control.buttons);

//	mavlink_msg_manual_control_pack(	quadrotor.mavlink_system.sysid,
//										quadrotor.mavlink_system.compid,
//										msg,
//										quadrotor.mavlink_system.sysid,
//										quadrotor.mavlink_control.x,
//										quadrotor.mavlink_control.y,
//										quadrotor.mavlink_control.z,
//										quadrotor.mavlink_control.r,
//										quadrotor.mavlink_control.buttons);

	/*
	 * PARA ALTITUDE TEST */
//	mavlink_msg_manual_control_pack(	quadrotor.mavlink_system.sysid,
//										quadrotor.mavlink_system.compid,
//										msg,
//										quadrotor.mavlink_system.sysid,
//										quadrotor.sv.motorOutput[0], //quadrotor.mavlink_control.x,
//										(int16_t) (quadrotor.sv.setpoint[ALTITUDE] * 1000), //quadrotor.mavlink_control.y,
//										(int16_t) (quadrotor.sv.altitude * 1000), //quadrotor.mavlink_control.z,
//										(int16_t) quadrotor.mavlink_control.z, //quadrotor.mavlink_control.r,
//										quadrotor.mavlink_control.buttons);
	return mavlink_msg_to_send_buffer( buf, msg );
}

inline uint16_t quadrotor_msg_attitude_quaternion ( mavlink_message_t * msg, uint8_t * buf )
{
	#if TEST_TELEMETRIA == TEST_TELEMETRIA__NONE
		mavlink_msg_attitude_quaternion_pack(	quadrotor.mavlink_system.sysid,
												quadrotor.mavlink_system.compid,
												msg,
												quadrotor.sv.time,
												quadrotor.sv.quaternion[0],
												quadrotor.sv.quaternion[1],
												quadrotor.sv.quaternion[2],
												quadrotor.sv.quaternion[3],
												quadrotor.sv.rate[ROLL],
												quadrotor.sv.rate[PITCH],
												quadrotor.sv.rate[YAW] );
	#elif TEST_TELEMETRIA == TEST_TELEMETRIA__ROLL
		mavlink_msg_attitude_quaternion_pack(	quadrotor.mavlink_system.sysid,
												quadrotor.mavlink_system.compid,
												msg,
												quadrotor.sv.time,
												quadrotor.sv.setpoint[ROLL],
												quadrotor.sv.rate[ROLL],
												quadrotor.sv.rateCtrlOutput[ROLL],
												0.5,
												quadrotor.sv.rate[ROLL],
												quadrotor.sv.rate[PITCH],
												quadrotor.sv.rate[YAW] );
	#elif TEST_TELEMETRIA == TEST_TELEMETRIA__PITCH
		mavlink_msg_attitude_quaternion_pack(	quadrotor.mavlink_system.sysid,
												quadrotor.mavlink_system.compid,
												msg,
												quadrotor.sv.time,
												quadrotor.sv.motorOutput[MOTOR1_ID],
												quadrotor.sv.motorOutput[MOTOR2_ID],
												quadrotor.sv.motorOutput[MOTOR3_ID],
												quadrotor.sv.motorOutput[MOTOR4_ID],
												quadrotor.sv.rate[YAW],
												quadrotor.sv.setpoint[YAW],
												quadrotor.sv.rateCtrlOutput[YAW] );
	#elif TEST_TELEMETRIA == TEST_TELEMETRIA__YAW
		mavlink_msg_attitude_quaternion_pack(	quadrotor.mavlink_system.sysid,
												quadrotor.mavlink_system.compid,
												msg,
												quadrotor.sv.time,
												quadrotor.sv.attitude[YAW],
												quadrotor.sv.setpoint[YAW],
												quadrotor.sv.attiCtrlOutput[YAW],
												quadrotor.sv.rateCtrlOutput[YAW],
												quadrotor.sv.rate[ROLL],
												quadrotor.sv.rate[PITCH],
												quadrotor.sv.rate[YAW] );
	#endif
	return mavlink_msg_to_send_buffer( buf, msg );
}
