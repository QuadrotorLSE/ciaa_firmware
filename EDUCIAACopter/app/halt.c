/*
 * halt.c
 *
 *  Created on: 13/09/2013
 *      Author: alan
 */

#include "DebugConsole.h"

void halt ( void )
{
	debug( "Halted!\n" );
	for(;;);
}

