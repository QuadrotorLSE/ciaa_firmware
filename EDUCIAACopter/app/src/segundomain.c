
//#include "chip.h"
//#include "board.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "qUART.h"
//#include "qESC.h"
//#include <stdint.h>
//#include "educiaa_gpio.h"
//
//#define EDUCIAA_NXP_LED   			0, 14
//#define EDUCIAA_UART				(3)
//
//static void 	Task_LED			( void *pvParameters );
//static void 	Task_UART_RX		( void *pvParameters );
//static void 	Task_UART_TX		( void *pvParameters );
//static void 	Task_PWM 			( void *pvParameters );
//
//void 			UART_Rx_Handler		(uint8_t * buff, size_t sz);
//
//char mensaje1[] = "Hola Mundo";
//char mensaje2[] = "Comienzo";
//char mensaje3[] = "TickTx";
//char mensaje4[] = "TickRx";
//
//int main(void)
//{
//	SystemCoreClockUpdate();
//	Board_Init();
//
//	if ( qUART_Init( EDUCIAA_UART, 57600, 8, QUART_PARITY_NONE, 1 ) != RET_OK )
//	{
//		while ( 1 ) {}
//	}
//	qUART_EnableTx( EDUCIAA_UART );
//
//	xTaskCreate( Task_PWM,		"Task_PWM",		configMINIMAL_STACK_SIZE,	NULL,	(tskIDLE_PRIORITY + 1UL),	(TaskHandle_t *) NULL );
//	xTaskCreate( Task_LED,		"Task_LED",		configMINIMAL_STACK_SIZE,	NULL,	(tskIDLE_PRIORITY + 1UL),	(TaskHandle_t *) NULL );
//	xTaskCreate( Task_UART_RX,	"Task_UART_RX",	configMINIMAL_STACK_SIZE,	NULL,	(tskIDLE_PRIORITY + 1UL),	(TaskHandle_t *) NULL );
//	xTaskCreate( Task_UART_TX,	"Task_UART_TX",	configMINIMAL_STACK_SIZE,	NULL,	(tskIDLE_PRIORITY + 1UL),	(TaskHandle_t *) NULL );
//	vTaskStartScheduler();
//	return 1;
//}
//
///*
// * PWM
// */
//#define MOTOR1		&pwm[0]
//qPWM_Channel pwm[] = {
//		{ 0,	1 },
//};
//static void Task_PWM ( void *pvParameters )
//{
//	uint8_t estado  = 0;
//	uint16_t out = 0;
//
//	qESC_Init();
//	qESC_InitChannel( MOTOR1 );
//	while ( 1 )
//	{
//		qESC_SetOutput( MOTOR1, (uint16_t)out );
////		if ( EDUCIAA_GPIO_Get( EDUCIAA_NXP_LED ) )
////			EDUCIAA_GPIO_Clear( EDUCIAA_NXP_LED );
////		else
////			EDUCIAA_GPIO_Set( EDUCIAA_NXP_LED );
//		vTaskDelay( configTICK_RATE_HZ );
//	}
//}
//
///***********************************************************************************************/
//
//static void Task_LED ( void *pvParameters )
//{
//	EDUCIAA_GPIO_Init();
//	while ( 1 )
//	{
//		if ( EDUCIAA_GPIO_Get( EDUCIAA_NXP_LED ) )
//			EDUCIAA_GPIO_Clear( EDUCIAA_NXP_LED );
//		else
//			EDUCIAA_GPIO_Set( EDUCIAA_NXP_LED );
//		vTaskDelay( configTICK_RATE_HZ );
//	}
//}
//
//static void Task_UART_TX ( void * pvParameters )
//{
//	qUART_Send( EDUCIAA_UART, (uint8_t*) mensaje1, (size_t) sizeof( mensaje1 ) );
//	qUART_Send( EDUCIAA_UART, (uint8_t*) mensaje2, (size_t) sizeof( mensaje2 ) );
//	while ( 1 )
//	{
//		qUART_Send( EDUCIAA_UART, (uint8_t*) mensaje3, (size_t) sizeof( mensaje3 ) );
//		vTaskDelay( configTICK_RATE_HZ * 5 );
//	}
//}
//
//uint8_t 	buffout[200];
//uint32_t 	buffsize = 0;
//static void Task_UART_RX ( void * pvParameters )
//{
//	while ( qUART_Status(EDUCIAA_UART) != DEVICE_READY );
//    qUART_Register_RBR_Callback( EDUCIAA_UART, UART_Rx_Handler );
//    qUART_EnableRx( EDUCIAA_UART );
//	while ( 1 )
//	{
//		if ( buffsize )
//		{
//			qUART_Send( EDUCIAA_UART, (uint8_t*) buffout, (size_t) buffsize );
//			buffsize = 0;
//		}
//		qUART_Send( EDUCIAA_UART, (uint8_t*) mensaje4, (size_t) sizeof( mensaje4 ) );
//		vTaskDelay( configTICK_RATE_HZ * 2 );
//	}
//}
//
//inline void UART_Rx_Handler ( uint8_t* buff, size_t sz )
//{
//	uint32_t 	i;
//	static portBASE_TYPE xHigherPriorityTaskWoken;
//	xHigherPriorityTaskWoken = pdFALSE;
//	for ( i = 0; i < sz; i++ )
//	{
//		buffout[buffsize + i] = buff[i];
//	}
//	buffsize += sz;
//	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
//}
//
