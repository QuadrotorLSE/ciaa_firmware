/*
 * hardware_init.c
 *
 *  Created on: 13/09/2013
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
#include "qI2C.h"
#include "parameters.h"
#include "leds.h"
#include "bmp085.h"
#include "HMC5883L.h"
#include "eMPL/inv_mpu.h"
#include "eMPL/inv_mpu_dmp_motion_driver.h"
#include "math.h"
#include "qESC.h"
#include "educiaa_gpio.h"

//TODO: Sacar esto de aca!
void MAVLink_Heartbeat	( void* );
void Communications		( void* );
void DataCollection		( void* );
void Distance			( void* );
void Telemetry			( void* );

void hardware_init ( void* p )
{
	uint16_t i;

	qESC_Init();
	qESC_InitChannel( MOTOR1 );
	qESC_InitChannel( MOTOR2 );
	qESC_InitChannel( MOTOR3 );
	qESC_InitChannel( MOTOR4 );

	vTaskDelay( 5000/portTICK_RATE_MS );
	if ( qUART_Init( UART_GROUNDCOMM, 57600, 8, QUART_PARITY_NONE, 1 ) != RET_OK )
	{
		halt();
	}
	qUART_EnableTx( UART_GROUNDCOMM );

	debug( "Inicializando hardware\r\n" );

	debug( "Inicializando interfase UART...[OK]\r\n" );

	EDUCIAA_GPIO_Init();
	EDUCIAA_GPIO_Clear( EDUCIAA_NXP_LEDR );
	EDUCIAA_GPIO_Clear( EDUCIAA_NXP_LEDG );
	EDUCIAA_GPIO_Clear( EDUCIAA_NXP_LEDB );
	EDUCIAA_GPIO_Clear( EDUCIAA_NXP_LED1 );
	EDUCIAA_GPIO_Clear( EDUCIAA_NXP_LED2 );
	EDUCIAA_GPIO_Clear( EDUCIAA_NXP_LED3 );
	debug( "Inicializando interfase GPIO...[OK]\r\n" );


	debug( "Inicializando interfase I2C..." );
	if ( qI2C_Init() != SUCCESS )
	{
		debug( "[ERROR]\r\n" );
		halt();
	}
	debug( "[OK]\r\n" );

	debug( "Inicializando MPU6050...\r\n" );
	debug( "Descargando firmware MPU6050 eMPL..." );
	if ( mpu_init( NULL ) == 0 )
	{
		debug( "[OK]\r\n" );
	}
	else
	{
		debug( "[ERROR]\r\n" );
		halt();
	}

	debug( "Configurando MPU6050 IMU..." );
	/* Get/set hardware configuration. Start gyro. */
	/* Wake up all sensors. */
	mpu_set_sensors( INV_XYZ_GYRO | INV_XYZ_ACCEL );
	/* Push both gyro and accel data into the FIFO. */
	mpu_configure_fifo( INV_XYZ_GYRO | INV_XYZ_ACCEL );
	mpu_set_sample_rate( 200 );
	dmp_load_motion_driver_firmware();
	mpu_set_gyro_fsr( 2000 );
	mpu_set_accel_fsr( 2 );
	mpu_set_lpf( 150 );
	dmp_enable_feature( DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL );
	dmp_set_fifo_rate( 200 );
	dmp_enable_gyro_cal( 1 );
	mpu_set_dmp_state( 1 );

	// TODO: estaria bueno que esto este adentro del driver (por ahi)
	// GPIO0.4 as input with interrupt
//	GPIO_SetDir( 0, (1<<4), 0 );
//	GPIO_IntCmd( 0, (1<<4), 1 );
//	GPIO_ClearInt( 0,(1<<4) );
//	NVIC_SetPriority( EINT3_IRQn, 6 );
//	NVIC_DisableIRQ( EINT3_IRQn );
	Chip_SCU_PinMuxSet( 6, 4, (SCU_MODE_PULLUP | SCU_MODE_FUNC0 | SCU_MODE_INBUFF_EN) );
	Chip_GPIO_SetPinDIRInput( LPC_GPIO_PORT, 3, 3 );
	Chip_GPIOGP_SelectLowLevel( LPC_GPIOGROUP, 1, 3, 1 << 3 );
	Chip_GPIOGP_EnableGroupPins( LPC_GPIOGROUP, 1, 3, 1 << 3 );
	Chip_GPIOGP_SelectAndMode( LPC_GPIOGROUP, 1 );
	Chip_GPIOGP_SelectEdgeMode( LPC_GPIOGROUP, 1 );
	NVIC_DisableIRQ( GINT1_IRQn );
	debug( "[OK]\r\n" );

	//=========================================================================
	quadrotor.mavlink_system.state = MAV_STATE_CALIBRATING; // TODO: Generar una macro para cabiar el estado del Quad
	quadrotor.sv.floor_pressure = 0.0; // presion a nivel del suelo
/*
	for (i=0;i<100;i++){
		BMP085_GetTemperature();
		quadrotor.sv.floor_pressure += BMP085_GetPressure()/100.0;
		vTaskDelay(10/portTICK_RATE_MS);
	}
*/
	debug("\r\nHardware [OK]\r\n\r\n");

	debug("Inicializando controladores\r\n");

	global_data.param[GP_YAW_SP_MAX]		= 80.0;
	global_data.param[GP_ROLL_SP_MAX]		= 60.0;
	global_data.param[GP_PITCH_SP_MAX]		= 60.0;
	global_data.param[GP_ALTI_DELTA]		= 0.00045;

	global_data.param[GP_YAW_LOOP]		= 1;
	global_data.param[GP_ROLL_LOOP]		= 2;
	global_data.param[GP_PITCH_LOOP]	= 2;

	// -------------------------------------------------------- rate controller
	for (i=0;i<3;i++){
		quadrotor.rateController[i].AntiWindup 		= ENABLED;
		quadrotor.rateController[i].Bumpless 		= ENABLED;
		quadrotor.rateController[i].Mode 			= AUTOMATIC;
		quadrotor.rateController[i].OutputMax 		= 1.0;
		quadrotor.rateController[i].OutputMin 		= -1.0;
		quadrotor.rateController[i].Ts 				= 0.005;
		quadrotor.rateController[i].b 				= 1.0;
		quadrotor.rateController[i].c 				= 0;
		qPID_Init(&quadrotor.rateController[i]);
	}
	// -------------------------------------------------------- atti controller
	for (i=0;i<3;i++){
		quadrotor.attiController[i].AntiWindup 		= ENABLED;
		quadrotor.attiController[i].Bumpless 		= ENABLED;
		quadrotor.attiController[i].Mode 			= AUTOMATIC;
		quadrotor.attiController[i].OutputMax 		= 250.0;
		quadrotor.attiController[i].OutputMin 		= -250.0;
		quadrotor.attiController[i].Ts 				= 0.005;
		quadrotor.attiController[i].b 				= 1.0;
		quadrotor.attiController[i].c 				= 1.0;
		qPID_Init(&quadrotor.attiController[i]);
	}
	// -------------------------------------------------------- altitude controller
	quadrotor.altitudeController.Bumpless 		= ENABLED;
	quadrotor.altitudeController.Mode 			= MANUAL;//AUTOMATIC;
	quadrotor.altitudeController.OutputMax 		= 0.9;
	quadrotor.altitudeController.OutputMin 		= 0.0;
	quadrotor.altitudeController.Ts 			= 0.005;
	quadrotor.altitudeController.b 				= 1.0;
	quadrotor.altitudeController.c 				= 0.0;
	qPID_Init(&quadrotor.altitudeController);
	// ======================================================== rate param
	// -------------------------------------------------------- roll
	global_data.param[RATE_ROLL_K]		= 0.007; // 0.01;
	global_data.param[RATE_ROLL_TI]		= 1.0 / 0.007;
	global_data.param[RATE_ROLL_TD]		= 0.0;
	global_data.param[RATE_ROLL_ND]		= 5.0;
	// -------------------------------------------------------- pitch
	global_data.param[RATE_PITCH_K]		= 0.007;
	global_data.param[RATE_PITCH_TI]	= 1.0 / 0.007;
	global_data.param[RATE_PITCH_TD]	= 0.0;
	global_data.param[RATE_PITCH_ND]	= 5.0;
	// -------------------------------------------------------- yaw
	global_data.param[RATE_YAW_K]		= 0.007;
	global_data.param[RATE_YAW_TI]		= 1.0 / 0.007;
	global_data.param[RATE_YAW_TD]		= 0.0;
	global_data.param[RATE_YAW_ND]		= 5;
	// ======================================================== atti param
	// -------------------------------------------------------- roll
	global_data.param[ATTI_ROLL_K]		= 1.0;
	global_data.param[ATTI_ROLL_TI]		= 99999.0;
	global_data.param[ATTI_ROLL_TD]		= 0.0;
	global_data.param[ATTI_ROLL_ND]		= 4.0;
	// -------------------------------------------------------- pitch
	global_data.param[ATTI_PITCH_K]		= 1.0;
	global_data.param[ATTI_PITCH_TI]	= 99999.0;
	global_data.param[ATTI_PITCH_TD]	= 0.0;
	global_data.param[ATTI_PITCH_ND]	= 4.0;
	// -------------------------------------------------------- yaw
	global_data.param[ATTI_YAW_K]		= 1;
	global_data.param[ATTI_YAW_TI]		= 99999.0;
	global_data.param[ATTI_YAW_TD]		= 0.0;
	global_data.param[ATTI_YAW_ND]		= 4.0;
	// ======================================================== alti param
	global_data.param[ALTI_K]			= 0.5;
	global_data.param[ALTI_TI]			= 1 / 1.50;
	global_data.param[ALTI_TD]			= 0.0;
	global_data.param[ALTI_ND]			= 5;

	debug("\r\nControladores [OK]\r\n\r\n");

	//=========================================================================
	//quadrotor.mavlink_system.state = MAV_STATE_STANDBY;

	quadrotor.mavlink_system.state = MAV_STATE_ACTIVE;
	quadrotor.mavlink_system.mode |= MAV_MODE_FLAG_SAFETY_ARMED;

	debug("( Sistema activo )\r\n");
	debug("( Desarmado )\r\n");

	quadrotor.mode = ESC_STANDBY;
	xTaskCreate( MAVLink_Heartbeat,	"HEARTBEAT", 	300, NULL, tskIDLE_PRIORITY + 2, NULL );
	xTaskCreate( Communications, 	"COMMS", 		300, NULL, tskIDLE_PRIORITY + 1, NULL );
	xTaskCreate( DataCollection,	"DATCOL", 		500, NULL, tskIDLE_PRIORITY + 4, NULL );
	xTaskCreate( Telemetry,			"TLM", 			300, NULL, tskIDLE_PRIORITY + 3, NULL );

	EDUCIAA_GPIO_Set( EDUCIAA_NXP_LED1 ); // sistema en funcionamiento

	for (;;)
	{
		if (quadrotor.mode == ESC_ARMED)
		{
//			for ( i = 0; i < 4; i++ )
//			{
				EDUCIAA_GPIO_Set( EDUCIAA_NXP_LEDR );
				EDUCIAA_GPIO_Clear( EDUCIAA_NXP_LEDB );
//				vTaskDelay(50/portTICK_RATE_MS);
//				EDUCIAA_GPIO_Clear( EDUCIAA_NXP_LEDR );
//				vTaskDelay(50/portTICK_RATE_MS);
//			}
		}
		else
		{
			EDUCIAA_GPIO_Clear( EDUCIAA_NXP_LEDR );
			EDUCIAA_GPIO_Set( EDUCIAA_NXP_LEDB );
		}
		vTaskDelay(500/portTICK_RATE_MS);
	}
}

