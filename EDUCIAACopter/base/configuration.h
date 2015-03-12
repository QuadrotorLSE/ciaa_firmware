/*
 * configuration.h
 *
 *  Created on: 01/02/2013
 *      Author: alan
 */

#ifndef EDUCIAACOPTER_BASE_CONFIGURATION_H_
#define EDUCIAACOPTER_BASE_CONFIGURATION_H_

#define RAM_TRACE			1
#define UART_CONSOLE		2

#define GYRO_RAW	1
#define GYRO_MPU	2

//#define DEBUG_LEVEL	UART_CONSOLE
//#define DEBUG_LEVEL RAM_TRACE
#define DEBUG_LEVEL UART_CONSOLE

#define GYRO_MODE	GYRO_RAW
#define TLM_PERIOD	20

#endif /* EDUCIAACOPTER_BASE_CONFIGURATION_H_ */
