/*
 * data_collection.c

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
#include "parameters.h"

#include "math.h"
#include "HMC5883L.h"
#include "eMPL/inv_mpu.h"
#include "eMPL/inv_mpu_dmp_motion_driver.h"
#include "qESC.h"
#include "educiaa_gpio.h"

enum
{
	_ALTITUDE_MODE__LIBRE,
	_ALTITUDE_MODE__INCREMENTAL,
	_ALTITUDE_MODE__DESPEGUE,
} altitude_mode;

#define PRESCALER_VALUE 10
xSemaphoreHandle mpuSempahore;
uint8_t prescaler = PRESCALER_VALUE;
float z_bias;

#define PI 3.14159265359

#define ATTI_THRESHOLD 3.0
float atti_bias[3];
float control[4]={0.0};
float yaw_sp_max;
float roll_sp_max;
float pitch_sp_max;
float alti_delta;

#define qxq(n,m)		(q1[(n)]*q1[(m)])
#define qe2(n)			(q1[(n)]*q1[(n)])

uint8_t MPU6050_dmpGetEuler ( float *euler, int32_t q[] )
{
	float q1[4];
	uint8_t i;

	for ( i = 0; i < 4; i++ )
	{
		q1[i] = ((float) (q[i]>>16)) / 16384.0f;
		quadrotor.sv.quaternion[i] = q1[i];
	}

	euler[ROLL] 	= atan2( 2*(qxq(0,1) + qxq(2,3)), qe2(0) - qe2(1) - qe2(2) + qe2(3) );
	euler[PITCH] 	=  asin( 2*(qxq(0,2) - qxq(3,1)) );
	euler[YAW] 		= atan2( 2*(qxq(0,3) + qxq(1,2)), qe2(0) + qe2(1) - qe2(2) - qe2(3) );

	return 0;
}

float map ( long x, long in_min, long in_max, float out_min, float out_max )
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void GINT1_IRQHandler ( void )
{
	static signed portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
	Chip_GPIOGP_ClearIntStatus( LPC_GPIOGROUP, 1 );
	if ( mpuSempahore != NULL )
	{
		xSemaphoreGiveFromISR(mpuSempahore,&xHigherPriorityTaskWoken);
	}
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

#define rad2grad( rad )		( (rad) * 180.0 / PI )

inline void DataCollection_Data_Input_Stage 			( float scale );
inline void DataCollection_Biasing 						( void );
inline void DataCollection_Control_Output_Stage 		( void );
inline void DataCollection_Controller_Processing_Stage 	( void );
inline void DataCollection_Controller_Output_Stage 		( void );

void DataCollection ( void *p )
{
	float scale;

	debug( "\r\nInicializando DataCollection\r\n" );
	mpu_get_gyro_sens( &scale );
	vSemaphoreCreateBinary( mpuSempahore );
	xSemaphoreTake( mpuSempahore, 0 );
	NVIC_EnableIRQ( GINT1_IRQn );
	prescaler = PRESCALER_VALUE;
	altitude_mode = _ALTITUDE_MODE__LIBRE;
	debug( "\r\nDataCollection [OK]\r\n" );

	while ( 1 )
	{
		// Wait here for MPU DMP interrupt at 200Hz
		xSemaphoreTake( mpuSempahore, portMAX_DELAY ); //FIXME: instead of portMAX it would be nice to have a time out for errors
		quadrotor.sv.time = xTaskGetTickCount() / portTICK_RATE_MS;
		DataCollection_Data_Input_Stage( scale );
		DataCollection_Biasing();
		DataCollection_Control_Output_Stage();
		DataCollection_Controller_Processing_Stage();
		DataCollection_Controller_Output_Stage();
	}
}

inline void DataCollection_Data_Input_Stage ( float scale )
{
	int32_t 	gyro[3];
	int32_t 	accel[3];
	int32_t 	quat[4];
	float 		atti_buffer[3];
	int32_t 	sensors;
	uint32_t	more;

	//-----------------------------------------------------------------------
	// Data input stage
	//-----------------------------------------------------------------------
	portENTER_CRITICAL();
	dmp_read_fifo( gyro, accel, (long *)quat, NULL, &sensors, &more );
	portEXIT_CRITICAL();

	MPU6050_dmpGetEuler( atti_buffer, quat );

	quadrotor.sv.attitude[ROLL]		=  rad2grad( atti_buffer[ROLL] );
	quadrotor.sv.attitude[PITCH]	=  rad2grad( atti_buffer[PITCH] );
	quadrotor.sv.attitude[YAW]		=  rad2grad( atti_buffer[YAW] );

	quadrotor.sv.rate[ROLL] 		= gyro[ROLL] 	/ scale; // [º/s]
	quadrotor.sv.rate[PITCH] 		= gyro[PITCH] 	/ scale; // [º/s]
	quadrotor.sv.rate[YAW] 			= gyro[YAW] 	/ scale; // [º/s]
}

inline void DataCollection_Biasing ( void )
{
	uint32_t i;
	//-----------------------------------------------------------------------
	// Biasing
	//-----------------------------------------------------------------------
	if ( (quadrotor.mavlink_control.buttons & BTN_L3) != 0 && (quadrotor.mavlink_control.buttons & BTN_R3) != 0 )
	{
		debug("( Set Point )\r\n");
		debug("( ALTI Libre )\r\n");
		altitude_mode = _ALTITUDE_MODE__LIBRE;

		yaw_sp_max			= global_data.param[GP_YAW_SP_MAX];
		roll_sp_max			= global_data.param[GP_ROLL_SP_MAX];
		pitch_sp_max		= global_data.param[GP_PITCH_SP_MAX];
		alti_delta			= global_data.param[GP_ALTI_DELTA];

		atti_bias[ROLL]		= quadrotor.sv.attitude[ROLL];
		atti_bias[PITCH] 	= quadrotor.sv.attitude[PITCH];
		atti_bias[YAW] 		= quadrotor.sv.attitude[YAW];
		// -------------------------------------------------------- rate roll
		quadrotor.rateController[ROLL].K	= global_data.param[RATE_ROLL_K];
		quadrotor.rateController[ROLL].Ti 	= global_data.param[RATE_ROLL_TI];
		quadrotor.rateController[ROLL].Td 	= global_data.param[RATE_ROLL_TD];
		quadrotor.rateController[ROLL].Nd	= global_data.param[RATE_ROLL_ND];
		qPID_Init( &quadrotor.rateController[ROLL] );
		// -------------------------------------------------------- rate pitch
		quadrotor.rateController[PITCH].K	= global_data.param[RATE_PITCH_K];
		quadrotor.rateController[PITCH].Ti 	= global_data.param[RATE_PITCH_TI];
		quadrotor.rateController[PITCH].Td 	= global_data.param[RATE_PITCH_TD];
		quadrotor.rateController[PITCH].Nd	= global_data.param[RATE_PITCH_ND];
		qPID_Init( &quadrotor.rateController[PITCH] );
		// -------------------------------------------------------- rate yaw
		quadrotor.rateController[YAW].K		= global_data.param[RATE_YAW_K];
		quadrotor.rateController[YAW].Ti 	= global_data.param[RATE_YAW_TI];
		quadrotor.rateController[YAW].Td 	= global_data.param[RATE_YAW_TD];
		quadrotor.rateController[YAW].Nd	= global_data.param[RATE_YAW_ND];
		qPID_Init( &quadrotor.rateController[YAW] );
		// -------------------------------------------------------- atti roll
		quadrotor.attiController[ROLL].K	= global_data.param[ATTI_ROLL_K];
		quadrotor.attiController[ROLL].Ti 	= global_data.param[ATTI_ROLL_TI];
		quadrotor.attiController[ROLL].Td 	= global_data.param[ATTI_ROLL_TD];
		quadrotor.attiController[ROLL].Nd	= global_data.param[ATTI_ROLL_ND];
		qPID_Init( &quadrotor.attiController[ROLL] );
		// -------------------------------------------------------- atti pitch
		quadrotor.attiController[PITCH].K	= global_data.param[ATTI_PITCH_K];
		quadrotor.attiController[PITCH].Ti 	= global_data.param[ATTI_PITCH_TI];
		quadrotor.attiController[PITCH].Td 	= global_data.param[ATTI_PITCH_TD];
		quadrotor.attiController[PITCH].Nd	= global_data.param[ATTI_PITCH_ND];
		qPID_Init( &quadrotor.attiController[PITCH] );
		// -------------------------------------------------------- atti yaw
		quadrotor.attiController[YAW].K		= global_data.param[ATTI_YAW_K];
		quadrotor.attiController[YAW].Ti 	= global_data.param[ATTI_YAW_TI];
		quadrotor.attiController[YAW].Td 	= global_data.param[ATTI_YAW_TD];
		quadrotor.attiController[YAW].Nd	= global_data.param[ATTI_YAW_ND];
		qPID_Init( &quadrotor.attiController[YAW] );
		// -------------------------------------------------------- alti
		quadrotor.altitudeController.K		= global_data.param[ALTI_K];
		quadrotor.altitudeController.Ti 	= global_data.param[ALTI_TI];
		quadrotor.altitudeController.Td 	= global_data.param[ALTI_TD];
		quadrotor.altitudeController.Nd		= global_data.param[ALTI_ND];
		qPID_Init( &quadrotor.altitudeController );
	}

	if ( quadrotor.mode == ESC_ARMED )
	{
		/*
		 * FIXME:
		 * Quizás sea peligroso que este paso se realice aprentando solo un botón!!!
		 * Imagino la situación en vuelvo descendiento muy rápido,
		 * si se activa el modo sera dificil cambiar la velocidad de descenso.
		 */
		if ( (quadrotor.mavlink_control.buttons & BTN_TRIANGLE) != 0 )
		{
			altitude_mode = _ALTITUDE_MODE__INCREMENTAL;
			debug("( ALTI Incremental )\r\n");
		}
		else if ( (quadrotor.mavlink_control.buttons & BTN_SQUARE) != 0 )
		{
			debug("( ALTI Despegue )\r\n");
			altitude_mode = _ALTITUDE_MODE__DESPEGUE;
			quadrotor.sv.setpoint[ALTITUDE] = 200.0/K_Z;
		}
	}
	else
	{
		altitude_mode = _ALTITUDE_MODE__LIBRE;
	}
}

inline void DataCollection_Control_Output_Stage ( void )
{
	float altitude_setpoint;
	float altitude_incremento = 0;
	//-----------------------------------------------------------------------
	// Control Output stage
	//-----------------------------------------------------------------------
	quadrotor.sv.attitude[ROLL] 	-= atti_bias[ROLL];
	quadrotor.sv.attitude[PITCH] 	-= atti_bias[PITCH];
	quadrotor.sv.attitude[YAW] 		-= atti_bias[YAW];

	if ( global_data.param[GP_YAW_LOOP] == LOOP__CAS )
	{
		quadrotor.sv.setpoint[YAW] = 0.0;
		if ( (quadrotor.mavlink_control.buttons & BTN_L2) != 0 )
			quadrotor.sv.setpoint[YAW] = yaw_sp_max;
		else if ( (quadrotor.mavlink_control.buttons & BTN_R2) != 0 )
			quadrotor.sv.setpoint[YAW] = -yaw_sp_max;
	}
	else if ( global_data.param[GP_YAW_LOOP] == LOOP__SAS )
	{
//XXX: Cuaidado ahora L1 y R1 son los botones de hombre muerto, el control de SAS
		quadrotor.sv.setpoint[YAW] = 0.0;
		if ( (quadrotor.mavlink_control.buttons & BTN_L2) != 0 )
			quadrotor.sv.setpoint[YAW] = yaw_sp_max;
		else if ( (quadrotor.mavlink_control.buttons & BTN_R2) != 0 )
			quadrotor.sv.setpoint[YAW] = -yaw_sp_max;
	}
	else
	{
		quadrotor.sv.setpoint[YAW] 		= 0.0;
	}

	if ( global_data.param[GP_ROLL_LOOP] == LOOP__CAS )
	{
		if ( quadrotor.mavlink_control.y < -10 || 10 < quadrotor.mavlink_control.y )
			quadrotor.sv.setpoint[ROLL] = map( quadrotor.mavlink_control.y,	-1000,	1000,	-roll_sp_max,	roll_sp_max );
		else
			quadrotor.sv.setpoint[ROLL] = 0.0;
	}
	else if ( global_data.param[GP_ROLL_LOOP] == LOOP__SAS )
	{
		quadrotor.sv.setpoint[ROLL] 	= 0.0;
	}
	else
	{
		quadrotor.sv.setpoint[ROLL] 	= 0.0;
	}

	if ( global_data.param[GP_PITCH_LOOP] == LOOP__CAS )
	{
		if ( quadrotor.mavlink_control.x < -10 || 10 < quadrotor.mavlink_control.x )
			quadrotor.sv.setpoint[PITCH] = map( quadrotor.mavlink_control.x,	-1000,	1000,	-pitch_sp_max,	pitch_sp_max );
		else
			quadrotor.sv.setpoint[PITCH] = 0.0;
	}
	else if ( global_data.param[GP_PITCH_LOOP] == LOOP__SAS )
	{
		quadrotor.sv.setpoint[PITCH] 	= 0.0;
	}
	else
	{
		quadrotor.sv.setpoint[PITCH] 	= 0.0;
	}

	/*
	 * ALTITUDE
	 */
	if ( altitude_mode == _ALTITUDE_MODE__LIBRE )
	{
		altitude_setpoint = map( quadrotor.mavlink_control.z, -1000, 1000, -1.0, 1.0 );
	}
	else if ( altitude_mode == _ALTITUDE_MODE__INCREMENTAL )
	{
		altitude_setpoint = quadrotor.sv.setpoint[ALTITUDE];
		altitude_incremento = 0;
		if ( quadrotor.mavlink_control.z < -10 || 10 < quadrotor.mavlink_control.z )
			altitude_incremento = map( quadrotor.mavlink_control.z, -1000, 1000, -alti_delta, alti_delta );
		altitude_setpoint += altitude_incremento;
	}
	else if ( altitude_mode == _ALTITUDE_MODE__DESPEGUE )
	{
		altitude_setpoint = quadrotor.sv.setpoint[ALTITUDE];
		altitude_incremento = alti_delta; //0.0003;
		altitude_setpoint += altitude_incremento;
		if ( altitude_setpoint > (500.0/K_Z) )
			altitude_mode = _ALTITUDE_MODE__INCREMENTAL;
	}
	quadrotor.sv.setpoint[ALTITUDE] = altitude_setpoint;
}

float U[2], Y[2];

inline void DataCollection_Controller_Processing_Stage ( void )
{
	//-----------------------------------------------------------------------
	// Controller processing stage
	//-----------------------------------------------------------------------

	//----------------------------------------------------------------------- YAW
	if ( global_data.param[GP_YAW_LOOP] == LOOP__CAS )
	{
		quadrotor.sv.attiCtrlOutput[YAW]	= qPID_Procees( &quadrotor.attiController[YAW],		quadrotor.sv.setpoint[YAW],			quadrotor.sv.attitude[YAW] );
		quadrotor.sv.rateCtrlOutput[YAW] 	= qPID_Procees( &quadrotor.rateController[YAW],		quadrotor.sv.attiCtrlOutput[YAW],	quadrotor.sv.rate[YAW] );
	}
	else if ( global_data.param[GP_YAW_LOOP] == LOOP__SAS )
	{
		quadrotor.sv.attiCtrlOutput[YAW] 	= 0.0;
		quadrotor.sv.rateCtrlOutput[YAW] 	= qPID_Procees( &quadrotor.rateController[YAW],		quadrotor.sv.setpoint[YAW],			quadrotor.sv.rate[YAW] );
	}
	else
	{
		quadrotor.sv.attiCtrlOutput[YAW] 	= 0.0;
		quadrotor.sv.rateCtrlOutput[YAW] 	= 0.0;
	}

	//----------------------------------------------------------------------- ROLL
	if ( global_data.param[GP_ROLL_LOOP] == LOOP__CAS )
	{
		quadrotor.sv.attiCtrlOutput[ROLL] 	= qPID_Procees( &quadrotor.attiController[ROLL],	quadrotor.sv.setpoint[ROLL],		quadrotor.sv.attitude[ROLL] );
		quadrotor.sv.rateCtrlOutput[ROLL] 	= qPID_Procees( &quadrotor.rateController[ROLL],	quadrotor.sv.attiCtrlOutput[ROLL],	quadrotor.sv.rate[ROLL] );
	}
	else if ( global_data.param[GP_ROLL_LOOP] == LOOP__SAS )
	{
		quadrotor.sv.attiCtrlOutput[ROLL] 	= 0.0;
		quadrotor.sv.rateCtrlOutput[ROLL] 	= qPID_Procees( &quadrotor.rateController[ROLL], 	quadrotor.sv.setpoint[ROLL], 		quadrotor.sv.rate[ROLL] );
	}
	else
	{
		quadrotor.sv.attiCtrlOutput[ROLL] 	= 0.0;
		quadrotor.sv.rateCtrlOutput[ROLL] 	= 0.0;
	}

	//----------------------------------------------------------------------- PITCH
	if ( global_data.param[GP_PITCH_LOOP] == LOOP__CAS )
	{
		quadrotor.sv.attiCtrlOutput[PITCH] 	= qPID_Procees( &quadrotor.attiController[PITCH],	quadrotor.sv.setpoint[PITCH],		quadrotor.sv.attitude[PITCH] );
		quadrotor.sv.rateCtrlOutput[PITCH] 	= qPID_Procees( &quadrotor.rateController[PITCH],	quadrotor.sv.attiCtrlOutput[PITCH],	quadrotor.sv.rate[PITCH] );
	}
	else if ( global_data.param[GP_PITCH_LOOP] == LOOP__SAS )
	{
		quadrotor.sv.attiCtrlOutput[PITCH] 	= 0.0;
		quadrotor.sv.rateCtrlOutput[PITCH] 	= qPID_Procees( &quadrotor.rateController[PITCH],	quadrotor.sv.setpoint[PITCH],		quadrotor.sv.rate[PITCH] );
	}
	else
	{
		quadrotor.sv.attiCtrlOutput[PITCH] 	= 0.0;
		quadrotor.sv.rateCtrlOutput[PITCH] 	= 0.0;
	}

	// ALTITUDE
	quadrotor.sv.altitudeCtrlOutput 		= qPID_Procees( &quadrotor.altitudeController,		quadrotor.sv.setpoint[ALTITUDE],	quadrotor.sv.altitude );
}

inline void DataCollection_Controller_Output_Stage ( void )
{
	uint16_t 	i;
	//-----------------------------------------------------------------------
	// Controller Output stage
	//-----------------------------------------------------------------------
	control[ROLL] 		=  K_PHI 	* quadrotor.sv.rateCtrlOutput[ROLL];
	control[PITCH] 		=  K_THETA  * quadrotor.sv.rateCtrlOutput[PITCH];
	control[YAW] 		=  K_PSI 	* quadrotor.sv.rateCtrlOutput[YAW];
	control[ALTITUDE] 	=  K_Z 		* quadrotor.sv.altitudeCtrlOutput;

	/* Numero de motores
	 *    1     4
	 *     \   /
	 *      1-4
	 *      |A|
	 *      2-3
	 *     /   \
	 *    2     3
	 */

	quadrotor.sv.motorOutput[MOTOR1_ID] = ( control[ALTITUDE] - control[PITCH] + control[ROLL] + control[YAW] );
	quadrotor.sv.motorOutput[MOTOR2_ID] = ( control[ALTITUDE] + control[PITCH] + control[ROLL] - control[YAW] );
	quadrotor.sv.motorOutput[MOTOR3_ID] = ( control[ALTITUDE] + control[PITCH] - control[ROLL] + control[YAW] );
	quadrotor.sv.motorOutput[MOTOR4_ID] = ( control[ALTITUDE] - control[PITCH] - control[ROLL] - control[YAW] );

	if ( quadrotor.mode == ESC_ARMED )
	{
		#if MOTOR1_ESTADO == ESTADO__ON
			qESC_SetOutput( MOTOR1, quadrotor.sv.motorOutput[MOTOR1_ID] );
		#elif MOTOR1_ESTADO == ESTADO__OFF
			qESC_SetOutput( MOTOR1, 0 );
		#else
			#error MOTOR1_ESTADO : Indefinido
		#endif

		#if MOTOR2_ESTADO == ESTADO__ON
			qESC_SetOutput( MOTOR2, quadrotor.sv.motorOutput[MOTOR2_ID] );
		#elif MOTOR2_ESTADO == ESTADO__OFF
			qESC_SetOutput( MOTOR2, 0 );
		#else
			#error MOTOR2_ESTADO : Indefinido
		#endif

		#if MOTOR3_ESTADO == ESTADO__ON
			qESC_SetOutput( MOTOR3, quadrotor.sv.motorOutput[MOTOR3_ID] );
		#elif MOTOR3_ESTADO == ESTADO__OFF
			qESC_SetOutput( MOTOR3, 0 );
		#else
			#error MOTOR3_ESTADO : Indefinido
		#endif

		#if MOTOR4_ESTADO == ESTADO__ON
			qESC_SetOutput( MOTOR4, quadrotor.sv.motorOutput[MOTOR4_ID] );
		#elif MOTOR4_ESTADO == ESTADO__OFF
			qESC_SetOutput( MOTOR4, 0 );
		#else
			#error MOTOR4_ESTADO : Indefinido
		#endif
	}
	else
	{
		qESC_SetOutput( MOTOR1, 0 );
		qESC_SetOutput( MOTOR2, 0 );
		qESC_SetOutput( MOTOR3, 0 );
		qESC_SetOutput( MOTOR4, 0 );

		if ( 	( fabsf( quadrotor.sv.attitude[ROLL] ) 	<= ATTI_THRESHOLD ) &&
				( fabsf( quadrotor.sv.attitude[PITCH] )	<= ATTI_THRESHOLD ) &&
				( fabsf( quadrotor.sv.attitude[YAW] ) 	<= ATTI_THRESHOLD ) )
		{
			EDUCIAA_GPIO_Set( EDUCIAA_NXP_LED2 );
		}
		else
		{
			EDUCIAA_GPIO_Clear( EDUCIAA_NXP_LED2 );
		}
	}
}
