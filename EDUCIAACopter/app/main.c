/*
 * main.c
 *
 *  Created on: 08/09/2013
 *      Author: alan
 */

#include "chip.h"
#include "board.h"
#include "quadrotor.h"
#include "DebugConsole.h"
#include "qboard.h"
#include "types.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "mavlink.h"
#include "mavlink_bridge.h"
#include "qUART.h"
#include "math.h"

void hardware_init(void *);

void AppMain ( void )
{
	SystemCoreClockUpdate();
	quadrotor.mavlink_system.sysid 		= 6;
	quadrotor.mavlink_system.compid 	= 01;
	quadrotor.mavlink_system.type 		= MAV_TYPE_QUADROTOR;
	quadrotor.mavlink_system.state 		= MAV_STATE_BOOT;
	quadrotor.mavlink_system.mode 		= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	quadrotor.mavlink_system.nav_mode 	= MAV_AUTOPILOT_GENERIC;
	xTaskCreate( hardware_init, 	"HW_INIT", 		300, NULL, tskIDLE_PRIORITY + 1, NULL );
	vTaskStartScheduler();
	while(1);
}
