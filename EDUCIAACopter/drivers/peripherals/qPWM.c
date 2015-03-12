/*
 * pwm.c
 *
 *  Created on: 28/10/2012
 *      Author: alan
 */

#include "qPWM.h"
#include "chip.h"
#include "types.h"

uint32_t qPWM_TicksOut = 0, qPWM_Ticks1ms = 0;

Status qPWM_Init ( uint32_t uS_period )
{
	if ( qPWM_TicksOut == 0 )
	{
		Chip_SCTPWM_Init( LPC_SCT );
		Chip_SCTPWM_SetRate( LPC_SCT, (uint32_t)(1000000.0 / ((float)uS_period) ) );
		Chip_SCTPWM_Start( LPC_SCT );

		uint32_t ticks_total, ticks_1ms;
		ticks_total 	= Chip_SCTPWM_GetTicksPerCycle( LPC_SCT );
		qPWM_Ticks1ms 	= ticks_total / 20;
		qPWM_TicksOut 	= qPWM_Ticks1ms / 1000;
	}

	return SUCCESS;
}

Status qPWM_InitChannel ( qPWM_Channel* q )
{
	Status out = SUCCESS;
	static uint8_t index = 1;

	if ( qPWM_TicksOut )
	{
		/*
		 * TODO no hay chequedo de N index !!! por SCT
		 */
		q->index = index;
		index++;
		/*
		 * TODO solo estoy asignando los PWM que me interesan
		 * por ejemplo el SCT 5 tiene  el 2_11 y el 4_5 !!!! pensar una solución!!!
		 */
		switch ( q->channel )
		{
			case 0:
				Chip_SCU_PinMuxSet( 4, 2, (SCU_MODE_INACT | SCU_MODE_FUNC1) );
				Chip_SCTPWM_SetOutPin( LPC_SCT, q->index, q->channel );
				break;
			case 1:
				Chip_SCU_PinMuxSet( 4, 1, (SCU_MODE_INACT | SCU_MODE_FUNC1) );
				Chip_SCTPWM_SetOutPin( LPC_SCT, q->index, q->channel );
				break;
			case 2:
				Chip_SCU_PinMuxSet( 4, 4, (SCU_MODE_INACT | SCU_MODE_FUNC1) );
				Chip_SCTPWM_SetOutPin( LPC_SCT, q->index, q->channel );
				break;
			case 3:
				Chip_SCU_PinMuxSet( 4, 3, (SCU_MODE_INACT | SCU_MODE_FUNC1) );
				Chip_SCTPWM_SetOutPin( LPC_SCT, q->index, q->channel );
				break;
			case 4:
				Chip_SCU_PinMuxSet( 2, 12, (SCU_MODE_INACT | SCU_MODE_FUNC1) );
				Chip_SCTPWM_SetOutPin( LPC_SCT, q->index, q->channel );
				break;
			case 5:
				Chip_SCU_PinMuxSet( 4, 5, (SCU_MODE_INACT | SCU_MODE_FUNC1) );
				Chip_SCTPWM_SetOutPin( LPC_SCT, q->index, q->channel );
				break;
			default:
				out = ERROR;
				break;
		}
		Chip_SCTPWM_SetDutyCycle( LPC_SCT, q->index, 0 );
	}
	else
	{
		out = ERROR;
	}

	return out;
}

Status qPWM_SetDuty ( qPWM_Channel* q, uint32_t uS_duty )
{
	Status out = ERROR;
	if ( qPWM_TicksOut )
	{
		Chip_SCTPWM_SetDutyCycle( LPC_SCT, q->index, (qPWM_TicksOut * uS_duty) );
		out = SUCCESS;
	}
	return out;
}

