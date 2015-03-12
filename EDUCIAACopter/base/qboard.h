/*
 * board.h
 *
 *  Created on: 27/10/2012
 *      Author: alan
 */

#ifndef EDUCIAACOPTER_BASE_QBOARD_H_
#define EDUCIAACOPTER_BASE_QBOARD_H_

#include "leds.h"
//#include "qAnalog.h"
#include "qPWM.h"

#define EEPROM_ADDRESS				0xA0
#define MPU6050_ADDRESS				0xD0
#define HMC5883L_ADDRESS			0x3C
#define BMP085_ADDRESS				0xEE

#define	STATUS_LED					leds[0]
#define	REAR_RIGHT_LED				leds[1]
#define	REAR_LEFT_LED				leds[2]
#define	FRONT_RIGHT_LED				leds[3]
#define	FRONT_LEFT_LED				leds[4]
#define	EXTERNAL_1_LED				leds[5] //LED5 - P7
#define	EXTERNAL_2_LED				leds[6] //LED6 - P8
#define TOTAL_LEDS					7

#define EDUCIAA_NXP_LED1   	0, 14
#define EDUCIAA_NXP_LED2   	1, 11
#define EDUCIAA_NXP_LED3   	1, 12
#define EDUCIAA_NXP_LEDR   	5, 0
#define EDUCIAA_NXP_LEDG   	5, 1
#define EDUCIAA_NXP_LEDB   	5, 2


//#define TEMPERATURE_ANALOG			&analog[1]
//#define VOLTAGE_ANALOG				&analog[0]

#define MOTOR1						&pwm[1]
#define MOTOR2						&pwm[2]
#define MOTOR3						&pwm[0]
#define MOTOR4						&pwm[3]

#define UART_GROUNDCOMM				(3) //LPC_UART0

//extern qAnalogInput analog[];
extern qLed leds[];
extern qPWM_Channel pwm[];

/*
 *
 */
#define ESTADO__ON	(1)
#define ESTADO__OFF	(2)

#define PARAMETROS_ENVIO__ESTADO			(ESTADO__ON)
#define TELEMETRIA_ENVIO__ESTADO			(ESTADO__ON)
#define TELEMETRIA_ENVIO__attitude			(ESTADO__ON)
#define TELEMETRIA_ENVIO__vfr				(ESTADO__OFF)
#define TELEMETRIA_ENVIO__manual_control	(ESTADO__ON)
#define TELEMETRIA_ENVIO__quaternion		(ESTADO__OFF)

/* Numero de motores
 *       1
 *       |
 *      1-4
 *  2---|A|---4
 *      2-3
 *       |
 *       3
 *
 *
 *    1     4
 *     \   /
 *      1-4
 *      |A|
 *      2-3
 *     /   \
 *    2     3
 */

enum
{
	MOTOR1_ID = 0,
	MOTOR2_ID,
	MOTOR3_ID,
	MOTOR4_ID,
};

#define ROLL		(0)
#define PITCH		(1)
#define YAW			(2)
#define ALTITUDE	(3)
#define NONE		(99)

#define LOOP__SAS		(1)
#define LOOP__CAS		(2)
#define LOOP__NO		(3)
#define LOOP__UNITARIO	(4)

#define TEST_TELEMETRIA__NONE		(NONE)
#define TEST_TELEMETRIA__PITCH		(PITCH)
#define TEST_TELEMETRIA__ROLL		(ROLL)
#define TEST_TELEMETRIA__YAW		(YAW)

#define TEST_TELEMETRIA				(TEST_TELEMETRIA__PITCH)
/*
 * ESTADO_ON: 	gira
 * ESTADO_OFF: 	no gira
 */
#define MOTOR1_ESTADO	(ESTADO__OFF)
#define MOTOR2_ESTADO	(ESTADO__OFF)
#define MOTOR3_ESTADO	(ESTADO__OFF)
#define MOTOR4_ESTADO	(ESTADO__OFF)
/*
 * LOOP__SAS: 	control de velocidad
 * LOOP__CAS: 	control de posición
 * LOOP__NO: 	sin control
 */
#define YAW_LOOP		(LOOP__NO)
#define ROLL_LOOP		(LOOP__NO)
#define PITCH_LOOP		(LOOP__CAS)
/*
 *
 */

#endif /* EDUCIAACOPTER_BASE_QBOARD_H_ */
