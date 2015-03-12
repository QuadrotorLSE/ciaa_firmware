/*
 * bmp085HAL.c
 *
 *  Created on: 29/10/2012
 *      Author: alan
 */
//#include "lpc17xx_gpio.h"
//#include "types.h"
//#include "bmp085.h"
//#include "_bmp085.h"
//#include "qI2C.h"
//#include "math.h"
//
//#include "FreeRTOS.h"
//#include "task.h"
//
//static bmp085_t bmp;
//
//static char read( unsigned char device_addr,unsigned char register_addr, unsigned char * register_data,  unsigned char read_length ){
//	qI2C_Read(device_addr,register_data,register_addr,read_length);
//	return 0;
//}
//
//static char write(unsigned char device_addr,unsigned char register_addr, unsigned char * register_data, unsigned char write_length ){
//	qI2C_Write(device_addr,register_data,register_addr,write_length);
//	return 0;
//}
//
//
//float BMP085_GetPressure(){
//	unsigned long up = bmp085_get_up();
//	return bmp085_get_pressure(up)/100.00;
//}
//
//float BMP085_GetTemperature(){
//	int32_t temp;
//	unsigned long ut = bmp085_get_ut();
//	temp = bmp085_get_temperature(ut);
//	return temp/10.00;
//}
//
//// in Pascals
//float BMP085_CalculateAltitude(float sealevel, float actual){
//	return 44330.00 * (1.00 - pow(actual /sealevel,0.1903));
//}
//
//
//Status BMP085_TestConnection(){
//	if (bmp.chip_id==BMP085_CHIP_ID){
//		return SUCCESS;
//	}else{
//		return ERROR;
//	}
//}
//
//void delay(uint32_t t){
//	vTaskDelay(t/portTICK_RATE_MS);
//}
//
//Status BMP085_Init(){
//	// Reset pin as output
//	GPIO_SetDir(1,(1<<1),1);
//
//	// Short reset
//	GPIO_ClearValue(1,(1<<1));
//	delay(1);
//	GPIO_SetValue(1,(1<<1));
//	delay(10); //Datasheet spec T_start
//
//	bmp.bus_write = write;
//	bmp.bus_read = read;
//	bmp.delay_msec = delay;
//
//	bmp085_init(&bmp);
//
//	return SUCCESS;
//}
