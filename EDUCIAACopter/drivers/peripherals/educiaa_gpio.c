/*
 * educiaa_gpio.c
 *
 *  Created on: 15/2/2015
 *      Author: Seba
 */

#include "chip.h"
#include "board.h"
#include "educiaa_gpio.h"

int EDUCIAA_GPIO_Init ( void )
{
	Chip_GPIO_Init( LPC_GPIO_PORT );

	/*
	 * 			SIM		GPIO	FUN
	 * LED1		2.10	0[14]	0
	 * LED2		2.11	1[11]	0
	 * LED3		2.12	1[12]	0
	 * LEDR		2.0		5[0]	4
	 * LEDG		2.1		5[1]	4
	 * LEDB		2.2		5[2]	4
	 * TEC1		1.0		0[4]	0
	 * TEC2		1.1		0[8]	0
	 * TEC3		1.2		0[9]	0
	 * TEC4		1.6		1[9]	0
	 */

	// LED1
	Chip_SCU_PinMuxSet( 2, 10, (SCU_MODE_PULLDOWN | SCU_MODE_FUNC0) );
	Chip_GPIO_SetPinDIROutput( LPC_GPIO_PORT, 0, 14 );
	Chip_GPIO_ClearValue( LPC_GPIO_PORT, 0, (1UL << 14) );
	// LED2
	Chip_SCU_PinMuxSet( 2, 11, (SCU_MODE_PULLDOWN | SCU_MODE_FUNC0) );
	Chip_GPIO_SetPinDIROutput( LPC_GPIO_PORT, 1, 11 );
	Chip_GPIO_ClearValue( LPC_GPIO_PORT, 1, (1UL << 11) );
	// LED3
	Chip_SCU_PinMuxSet( 2, 12, (SCU_MODE_PULLDOWN | SCU_MODE_FUNC0) );
	Chip_GPIO_SetPinDIROutput( LPC_GPIO_PORT, 1, 12 );
	Chip_GPIO_ClearValue( LPC_GPIO_PORT, 1, (1UL << 12) );
	// LEDR
	Chip_SCU_PinMuxSet( 2, 0, (SCU_MODE_PULLDOWN | SCU_MODE_FUNC4) );
	Chip_GPIO_SetPinDIROutput( LPC_GPIO_PORT, 5, 0 );
	Chip_GPIO_ClearValue( LPC_GPIO_PORT, 5, (1UL << 0) );
	// LEDG
	Chip_SCU_PinMuxSet( 2, 1, (SCU_MODE_PULLDOWN | SCU_MODE_FUNC4) );
	Chip_GPIO_SetPinDIROutput( LPC_GPIO_PORT, 5, 1 );
	Chip_GPIO_ClearValue( LPC_GPIO_PORT, 5, (1UL << 1) );
	// LEDB
	Chip_SCU_PinMuxSet( 2, 2, (SCU_MODE_PULLDOWN | SCU_MODE_FUNC4) );
	Chip_GPIO_SetPinDIROutput( LPC_GPIO_PORT, 5, 2 );
	Chip_GPIO_ClearValue( LPC_GPIO_PORT, 5, (1UL << 2) );
	// TEC1
	Chip_SCU_PinMuxSet( 1, 0, (SCU_MODE_INACT | SCU_MODE_FUNC0 | SCU_MODE_INBUFF_EN) );
	Chip_GPIO_SetPinDIRInput( LPC_GPIO_PORT, 0, 4 );
	// TEC2
	Chip_SCU_PinMuxSet( 1, 1, (SCU_MODE_INACT | SCU_MODE_FUNC0 | SCU_MODE_INBUFF_EN) );
	Chip_GPIO_SetPinDIRInput( LPC_GPIO_PORT, 0, 8 );
	// TEC3
	Chip_SCU_PinMuxSet( 1, 2, (SCU_MODE_INACT| SCU_MODE_FUNC0 | SCU_MODE_INBUFF_EN) );
	Chip_GPIO_SetPinDIRInput( LPC_GPIO_PORT, 0, 9 );
	// TEC4
	Chip_SCU_PinMuxSet( 1, 6, (SCU_MODE_INACT | SCU_MODE_FUNC0 | SCU_MODE_INBUFF_EN) );
	Chip_GPIO_SetPinDIRInput( LPC_GPIO_PORT, 1, 9 );
}

int EDUCIAA_GPIO_Get ( uint8_t port, uint8_t pin )
{
	if ( Chip_GPIO_GetPinDIR( LPC_GPIO_PORT, port, pin ) )
	{
		if ( LPC_GPIO_PORT->SET[port] & (1UL <<  pin) )
			return 1;
		else
			return 0;
	}
	else
	{
		if ( Chip_GPIO_GetPinState( LPC_GPIO_PORT, port, pin ) )
			return 1;
		else
			return 0;
	}
}

int EDUCIAA_GPIO_Set ( uint8_t port, uint8_t pin )
{
	Chip_GPIO_SetValue( LPC_GPIO_PORT, port, (1UL << pin) );
}

int EDUCIAA_GPIO_Clear ( uint8_t port, uint8_t pin )
{
	Chip_GPIO_ClearValue( LPC_GPIO_PORT, port, (1UL << pin) );
}
