/***********************************************************************//**
 * @file		qUART.h
 * @brief		Contains all macro definitions and function prototypes
 * 				for the UART driver.
 * @version
 * @date
 * @author
 *************************************************************************/
/** @ingroup BSP */
/** @addtogroup qUART */
/*@{*/

#ifndef qUART_H_
#define qUART_H_

#include "chip.h"
#include "string.h"
#include "types.h"

typedef enum{
	QUART_PARITY_NONE=0,	/**< No parity is used */
	QUART_PARITY_ODD,		/**< Adds a 1 if the number of ones is not odd */
	QUART_PARITY_EVEN,		/**< Adds a 1 if the number of ones is not even */
}qUART_Parity_t;

ret_t 		qUART_Init					( uint8_t id, uint32_t BaudRate, uint8_t DataBits, qUART_Parity_t Parity, uint8_t StopBits );
ret_t 		qUART_DeInit				( uint8_t qUART_ID );
ret_t 		qUART_Register_RBR_Callback	( uint8_t id, void (*pf) ( uint8_t*, size_t sz ) );
uint32_t 	qUART_Send					( uint8_t qUART_ID, uint8_t* buff, size_t size );
ret_t 		qUART_SendByte				( uint8_t qUART_ID, uint8_t ch );
ret_t 		qUART_ReadByte				( uint8_t id, uint8_t* buffer );
ret_t 		qUART_EnableTx				( uint8_t id );
ret_t 		qUART_EnableRx				( uint8_t id );
ret_t 		qUART_DisableTx				( uint8_t id );
ret_t 		qUART_DisableRx				( uint8_t id );
status_t 	qUART_Status 				( uint8_t id );

#endif /* qUART_H_ */

/*@}*/
/*@}*/
