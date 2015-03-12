// API Interface
#include "qUART.h"

#define _GPDMA_CONN_UART_Tx 	(GPDMA_CONN_UART3_Tx)
#define _GPDMA_CONN_UART_Rx 	(GPDMA_CONN_UART3_Rx)
#define _qUART_0				LPC_USART0
#define _qUART_1				LPC_UART1
#define _qUART_2				LPC_USART2
#define _qUART_3				LPC_USART3
#define _qUART_MAX				(4)
#define _qUART__BUFF_SIZE		(400)
#define _qUART__BUFF_MAX		(2)

status_t 			qUARTStatus[_qUART_MAX] 	= {0}; /* DEVICE_NOT_READY */
volatile uint8_t 	rxBuff[_qUART__BUFF_MAX][_qUART__BUFF_SIZE];
volatile uint8_t 	txBuff[_qUART__BUFF_MAX][_qUART__BUFF_SIZE];
volatile uint8_t 	selectedRxBuff;
volatile uint8_t 	selectedTxBuff 				= 0;
volatile uint32_t	txBufferCount 				= 0;
volatile uint8_t	timerRunning 				= 0;

void (*RBR_Handler[_qUART_MAX]) ( uint8_t *,size_t sz ) = {NULL};
static LPC_USART_T* uarts[] = { _qUART_0, _qUART_1, _qUART_2, _qUART_3 };

typedef struct GPDMA_Cfg_s
{
	uint8_t 				ChannelNum;
	uint32_t 				src;
	uint32_t 				dst;
	GPDMA_FLOW_CONTROL_T 	TransferType;
	uint32_t 				Size;
} GPDMA_Cfg;
GPDMA_Cfg GPDMACfg_rx;
GPDMA_Cfg GPDMACfg_tx;

void 			qUART_flushBuffer 		( void );
inline void 	qUART_NextTxBuffer 		( void );
inline void 	qUART_NextRxBuffer 		( void );
void 			qUART_TxCopy 			( uint8_t* buff, size_t size );
void 			UARTx_IRQHandler 		( uint8_t id );
void 			UART_IntErr				( uint8_t id, uint8_t bLSErrType );

ret_t qUART_Init ( uint8_t id, uint32_t BaudRate, uint8_t DataBits, qUART_Parity_t Parity, uint8_t StopBits )
{
	// Check if the device wasn't initialized first
	if ( qUARTStatus[id] == DEVICE_READY )
		return RET_ALREADY_INIT;

	Chip_UART_Init( uarts[id] );
	Chip_UART_SetBaud( uarts[id], BaudRate );
	Chip_UART_ConfigData( uarts[id], (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT) ); /* Default 8-N-1 */
	Chip_UART_IntEnable( uarts[id], (UART_IER_RBRINT) );
	if 		( uarts[id] == _qUART_0 )	{	NVIC_SetPriority( USART0_IRQn, 6 );		NVIC_EnableIRQ( USART0_IRQn );		}
	else if ( uarts[id] == _qUART_1 )	{	NVIC_SetPriority( UART1_IRQn, 6 );		NVIC_EnableIRQ( UART1_IRQn );		}
	else if ( uarts[id] == _qUART_2 )	{	NVIC_SetPriority( USART2_IRQn, 6 );		NVIC_EnableIRQ( USART2_IRQn );		}
	else if ( uarts[id] == _qUART_3 )	{	NVIC_SetPriority( USART3_IRQn, 6 );		NVIC_EnableIRQ( USART3_IRQn );		}
	else								{	return RET_ERROR;															}
	Chip_UART_SetupFIFOS( uarts[id], (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_DMAMODE_SEL | UART_FCR_TRG_LEV3) );
	Chip_UART_TXEnable( uarts[id] );

	selectedRxBuff = 0;
	selectedTxBuff = 0;

	Chip_GPDMA_Init( LPC_GPDMA );

   	GPDMACfg_tx.ChannelNum 		= Chip_GPDMA_GetFreeChannel( LPC_GPDMA, _GPDMA_CONN_UART_Tx );
   	Chip_GPDMA_ChannelCmd( LPC_GPDMA, GPDMACfg_tx.ChannelNum, DISABLE );
   	GPDMACfg_tx.src 			= (uint32_t) txBuff[selectedTxBuff];
   	GPDMACfg_tx.dst 			= _GPDMA_CONN_UART_Tx;
   	GPDMACfg_tx.TransferType 	= GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA;
   	GPDMACfg_tx.Size 			= _qUART__BUFF_SIZE;

   	GPDMACfg_rx.ChannelNum 		= Chip_GPDMA_GetFreeChannel( LPC_GPDMA, _GPDMA_CONN_UART_Rx );
   	Chip_GPDMA_ChannelCmd( LPC_GPDMA, GPDMACfg_rx.ChannelNum, DISABLE );
   	GPDMACfg_rx.src 			= _GPDMA_CONN_UART_Rx;
   	GPDMACfg_rx.dst 			= (uint32_t) rxBuff[selectedRxBuff];
   	GPDMACfg_rx.TransferType 	= GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA;
   	GPDMACfg_rx.Size 			= _qUART__BUFF_SIZE;

   	Chip_GPDMA_Transfer( LPC_GPDMA, GPDMACfg_rx.ChannelNum, GPDMACfg_rx.src, GPDMACfg_rx.dst, GPDMACfg_rx.TransferType, GPDMACfg_rx.Size );
	Chip_GPDMA_ChannelCmd( LPC_GPDMA, GPDMACfg_rx.ChannelNum, ENABLE );

	Chip_TIMER_Init( LPC_TIMER1 );
	Chip_RGU_TriggerReset( RGU_TIMER1_RST );
	while (Chip_RGU_InReset( RGU_TIMER1_RST )) {}
	Chip_TIMER_Reset( LPC_TIMER1 );
	Chip_TIMER_PrescaleSet( LPC_TIMER1, 1 );
	Chip_TIMER_SetMatch( LPC_TIMER1, 1, 5000 );
	Chip_TIMER_ResetOnMatchEnable( LPC_TIMER1, 1 );
	Chip_TIMER_MatchEnableInt( LPC_TIMER1, 1 );
	NVIC_EnableIRQ( TIMER1_IRQn );
	NVIC_ClearPendingIRQ( TIMER1_IRQn );

	qUARTStatus[id] = DEVICE_READY;

	return RET_OK;
}
ret_t qUART_DeInit ( uint8_t id )
{
	if ( qUARTStatus[id] == DEVICE_READY )
	{
		Chip_UART_DeInit( uarts[id] );
		qUARTStatus[id] = DEVICE_NOT_READY;
		return RET_OK;
	}
	return RET_ERROR;
}
ret_t qUART_Register_RBR_Callback ( uint8_t id, void (*pf)(uint8_t *, size_t sz) )
{
	if ( pf == NULL )
		return RET_ERROR;

	RBR_Handler[id] = pf;
	return RET_OK;
}

status_t qUART_Status ( uint8_t id )
{
	return qUARTStatus[id];
}

uint32_t qUART_Send ( uint8_t id, uint8_t* buff, size_t size )
{
	if (timerRunning == 1)
	{
		timerRunning = 0;
		Chip_TIMER_Disable( LPC_TIMER1 );
	}

	// Chequeo si hay lugar en el buffer de transmision
	if ( (txBufferCount + size) >= _qUART__BUFF_SIZE )
	{
		// Si no hay lugar, mando el buffer viejo swapeo y lo meto en el nuevo
		qUART_flushBuffer();
	}
	// Si hay lugar lo meto adentro
	qUART_TxCopy( buff, size );

	// Chequeo si ahora con la nueva info llene el buffer
	if ( txBufferCount == (_qUART__BUFF_SIZE - 1) )
	{
		qUART_flushBuffer();
	}
	else
	{
		Chip_TIMER_Reset( LPC_TIMER1 );

		timerRunning = 1;
		Chip_TIMER_Enable( LPC_TIMER1 );
	}

	return RET_OK;
}

void qUART_TxCopy ( uint8_t* buff, size_t size )
{
	memcpy( &(txBuff[selectedTxBuff][txBufferCount] ), buff, size );
	txBufferCount += size;
}

void qUART_flushBuffer ( void )
{
	GPDMACfg_tx.src 	= (uint32_t) &txBuff[selectedTxBuff];
	GPDMACfg_tx.Size 	= txBufferCount;
	Chip_GPDMA_Transfer( LPC_GPDMA, GPDMACfg_tx.ChannelNum, GPDMACfg_tx.src, GPDMACfg_tx.dst, GPDMACfg_tx.TransferType, GPDMACfg_tx.Size );
	Chip_GPDMA_ChannelCmd( LPC_GPDMA, GPDMACfg_tx.ChannelNum, ENABLE );
	//XXX: NO hay chequeo de over run aca
	qUART_NextTxBuffer();
}

inline void qUART_NextTxBuffer ( void )
{
	if ( selectedTxBuff >= (_qUART__BUFF_MAX - 1) )
		selectedTxBuff = 0;
	else
		selectedTxBuff++;

	txBufferCount = 0;
}
inline void qUART_NextRxBuffer ( void )
{
	if ( selectedRxBuff >= (_qUART__BUFF_MAX - 1) )
		selectedRxBuff = 0;
	else
		selectedRxBuff++;
}

ret_t qUART_SendByte ( uint8_t id, uint8_t ch )
{
	qUART_Send( id, &ch, 1 );
}

ret_t qUART_ReadByte ( uint8_t id, uint8_t* buffer )
{

}

//===========================================================
// Handlers
//===========================================================
void UART0_IRQHandler ( void )
{
	UARTx_IRQHandler( 0 );
}

void UART1_IRQHandler ( void )
{
	UARTx_IRQHandler( 1 );
}

void UART2_IRQHandler ( void )
{
	UARTx_IRQHandler( 2 );
}

void UART3_IRQHandler ( void )
{
	UARTx_IRQHandler( 3 );
}

void UARTx_IRQHandler ( uint8_t id )
{
	uint32_t 	buffsize;
	uint32_t 	i = 0;
	uint8_t 	selectedBuff;

	for ( i = 0; i < 500; i++ )
	{
//		 ESTE delay hace que todo funcione,
//		 sino parece que no terminaba el transer y le
//		 cambiaba el banco antes de terminar y se rompia todo
//		 1000 no es mucho?
		// SABES QUE TENES RAZÓN !!!!
	}

	selectedBuff = selectedRxBuff;
	qUART_NextRxBuffer();
	Chip_GPDMA_ChannelCmd( LPC_GPDMA, GPDMACfg_rx.ChannelNum, DISABLE );
	// Desde aca hay solo una FIFO (16 bytes) de tiempo para encender el otro buffer
	buffsize = ( LPC_GPDMA->CH[GPDMACfg_rx.ChannelNum].DESTADDR ) - ((uint32_t) &(rxBuff[selectedBuff]));
	GPDMACfg_rx.dst = (uint32_t) &rxBuff[selectedRxBuff];
	Chip_GPDMA_Transfer( LPC_GPDMA, GPDMACfg_rx.ChannelNum, GPDMACfg_rx.src, GPDMACfg_rx.dst, GPDMACfg_rx.TransferType, GPDMACfg_rx.Size );
	// Hasta aca!
	Chip_GPDMA_ChannelCmd( LPC_GPDMA, GPDMACfg_rx.ChannelNum, ENABLE );

	if ( RBR_Handler[id] != NULL )
		(*RBR_Handler[id])( (uint8_t*) &rxBuff[selectedBuff], (size_t) buffsize);
}

void TIMER1_IRQHandler(void)
{
	if ( Chip_TIMER_MatchPending( LPC_TIMER1, 1 ) )
	{
		qUART_flushBuffer();
		timerRunning = 0;
		Chip_TIMER_Disable( LPC_TIMER1 );
	}
	Chip_TIMER_ClearMatch( LPC_TIMER1, 1 );
}

/*
 * ---------------------------------------------------------------------
 */
ret_t qUART_EnableTx ( uint8_t id )
{
	ret_t out = RET_OK;
	switch ( id )
	{
		case 0:		out = RET_ERROR;	break;
		case 1:		out = RET_ERROR;	break;
		case 2:		out = RET_ERROR;	break;
		case 3:		Chip_SCU_PinMuxSet( 2, 3, (SCU_MODE_PULLDOWN | SCU_MODE_FUNC2) ); break;
		default:	out = RET_ERROR; break;
	}
	return out;
}

ret_t qUART_DisableTx ( uint8_t id )
{
	ret_t out = RET_OK;
	switch ( id )
	{
		case 0:		out = RET_ERROR;	break;
		case 1:		out = RET_ERROR;	break;
		case 2:		out = RET_ERROR;	break;
		case 3:		Chip_SCU_PinMuxSet( 2, 3, (SCU_MODE_FUNC0) ); break;
		default:	out = RET_ERROR; break;
	}
	return out;
}

ret_t qUART_EnableRx ( uint8_t id )
{
	ret_t out = RET_OK;
	switch ( id )
	{
		case 0:		out = RET_ERROR;	break;
		case 1:		out = RET_ERROR;	break;
		case 2:		out = RET_ERROR;	break;
		case 3:		Chip_SCU_PinMuxSet( 2, 4, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC2) ); break;
		default:	out = RET_ERROR; break;
	}
	return out;
}

ret_t qUART_DisableRx ( uint8_t id )
{
	ret_t out = RET_OK;
	switch ( id )
	{
		case 0:		out = RET_ERROR;	break;
		case 1:		out = RET_ERROR;	break;
		case 2:		out = RET_ERROR;	break;
		case 3:		Chip_SCU_PinMuxSet( 2, 4, (SCU_MODE_FUNC0) ); break;
		default:	out = RET_ERROR; break;
	}
	return out;
}
