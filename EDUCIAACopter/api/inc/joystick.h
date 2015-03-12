/*
 * joystick.h
 *
 *  Created on: 13/01/2013
 *      Author: alan
 */

#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include <stdint.h>


/*
 * Joystick PS3
 */
//#define	BTN_SELECT	 	(1<<0)
//#define	BTN_L3			(1<<1)
//#define	BTN_R3			(1<<2)
//#define	BTN_START		(1<<3)
//#define	BTN_UP			(1<<4)
//#define	BTN_RIGHT		(1<<5)
//#define	BTN_DOWN		(1<<9)
//#define	BTN_LEFT		(1<<6)
//#define	BTN_LEFT2		(1<<7) // no funca el LEFT2!!!
//#define	BTN_RIGHT2		(1<<8)
//#define	BTN_LEFT1		(1<<10)
//#define	BTN_RIGHT1		(1<<11)
//#define	BTN_TRIANGLE	(1<<12)
//#define	BTN_CIRCLE		(1<<13)
//#define	BTN_CROSS		(1<<14)
//#define	BTN_SQUARE		(1<<15)
/*
 * Joystick RumblePower
 */
#define	BTN_TRIANGLE	(1<<0)
#define	BTN_CIRCLE		(1<<1)
#define	BTN_CROSS		(1<<2)
#define	BTN_SQUARE		(1<<3)
#define	BTN_L1			(1<<4)
#define	BTN_R1			(1<<5)
#define	BTN_L2			(1<<6)
#define	BTN_R2			(1<<7) // no funciona
#define	BTN_SELECT	 	(1<<8)
#define	BTN_START		(1<<9)
#define	BTN_L3			(1<<10)
#define	BTN_R3			(1<<11)

typedef struct{
	uint8_t x;
	uint8_t y;
}joystick_pad_t;

typedef struct{
	joystick_pad_t		left_pad;
	joystick_pad_t		right_pad;
	uint8_t 	L1;
	uint8_t 	L2;
	uint8_t 	R1;
	uint8_t 	R2;
	uint16_t 	buttons;
}joystick_t;


#endif /* JOYSTICK_H_ */
