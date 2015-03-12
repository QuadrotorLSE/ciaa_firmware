/*
 * board.c
 *
 *  Created on: 27/10/2012
 *      Author: alan
 */

#include <qboard.h>
#include "leds.h"
//#include "qAnalog.h"

qLed leds[] = {	{0,22,ACTIVE_HIGH},
				{1,28,ACTIVE_HIGH},
				{1,29,ACTIVE_HIGH},
				{4,28,ACTIVE_HIGH},
				{4,29,ACTIVE_HIGH},
				{1,22,ACTIVE_HIGH},
				{1,25,ACTIVE_HIGH},
				};

//qAnalogInput analog[] = {
//		{30,1,3,ADC_CHANNEL_4},
//		{31,1,3,ADC_CHANNEL_5},
//};

qPWM_Channel pwm[] = {
		{ 0,	0 },
		{ 0,	5 },
		{ 0,	2 },
		{ 0,	3 },
};
