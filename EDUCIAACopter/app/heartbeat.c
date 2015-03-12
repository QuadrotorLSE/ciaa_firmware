/*
 * heartbeat.c
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
#include "DebugConsole.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "mavlink.h"
#include "mavlink_bridge.h"
#include "qUART.h"

#define MAX_TASKS	5
char task_names[MAX_TASKS][20];
uint32_t task_usage[MAX_TASKS];

void MAVLink_Heartbeat ( void* p )
{
	portTickType xLastWakeTime;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_message_t msg;
	uint16_t len;

	debug("\r\nInicializando Heartbeat\r\n");
	xLastWakeTime = xTaskGetTickCount ();
	debug("\r\nHeartbeat [OK]\r\n");

	while ( 1 )
	{
		if ( qUART_Status( UART_GROUNDCOMM ) == DEVICE_READY )
		{
			mavlink_msg_heartbeat_pack(	quadrotor.mavlink_system.sysid,
										quadrotor.mavlink_system.compid,
										&msg,
										quadrotor.mavlink_system.type,
										quadrotor.mavlink_system.nav_mode,
										quadrotor.mavlink_system.mode,
										0,
										quadrotor.mavlink_system.state);

			len = mavlink_msg_to_send_buffer( buf, &msg );
			qUART_Send( UART_GROUNDCOMM, buf, len );
//			debug("oH\r\n");
		}
		vTaskDelayUntil( &xLastWakeTime, 300/portTICK_RATE_MS); // TODO: Chequear si con 500mseg de Heartbeat funciona bien!
	}
}


