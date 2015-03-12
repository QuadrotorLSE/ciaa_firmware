/*
 * educiaa_gpio.h
 *
 *  Created on: 15/2/2015
 *      Author: Seba
 */

#ifndef EDUCIAA_GPIO_H_
#define EDUCIAA_GPIO_H_

#include "chip.h"
#include "board.h"

int 	EDUCIAA_GPIO_Init 	( void );
int 	EDUCIAA_GPIO_Get 	( uint8_t port, uint8_t pin );
int 	EDUCIAA_GPIO_Set 	( uint8_t port, uint8_t pin );
int 	EDUCIAA_GPIO_Clear 	( uint8_t port, uint8_t pin );

#endif /* EDUCIAA_GPIO_H_ */
