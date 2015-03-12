

#include "qI2C.h"

#include <stdint.h>
#include <string.h>

#include "chip.h"

static I2CM_XFER_T  i2cmXferRec;

/**
* @brief  Initializes the I2C peripheral used to drive the MPU6050
* @param  None
* @return None
*/
Status qI2C_Init ( void )
{
	Chip_SCU_I2C0PinConfig( I2C0_STANDARD_FAST_MODE );
	Chip_I2C_Init( I2C0 );
	Chip_I2C_SetClockRate( I2C0, I2C_BITRATE);

	return SUCCESS;
}

/**
* @brief  Writes one byte to the  MPU6050.
* @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
* @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
* @param  writeAddr : address of the register in which the data will be written
* @return None
*/

Status qI2C_Write( uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr, uint16_t NumByteToWrite )
{

	uint8_t sendBuffer[NumByteToWrite+1];
	Status res;

	sendBuffer[0] = writeAddr;
	memcpy( &sendBuffer[1], pBuffer, NumByteToWrite );

//	I2C_M_SETUP_Type txsetup;
//	txsetup.sl_addr7bit = slaveAddr>>1;
//	txsetup.tx_data = sendBuffer;
//	txsetup.tx_length = sizeof(sendBuffer);
//	txsetup.rx_data = NULL;
//	txsetup.rx_length = 0;
//	txsetup.retransmissions_max = 3;
//	res = I2C_MasterTransferData(LPC_I2C1, &txsetup, I2C_TRANSFER_POLLING);

	i2cmXferRec.slaveAddr 	= slaveAddr>>1;
	i2cmXferRec.options 	= 0;
	i2cmXferRec.status 		= 0;
	i2cmXferRec.txSz 		= sizeof(sendBuffer);
	i2cmXferRec.rxSz 		= 0;
	i2cmXferRec.txBuff 		= sendBuffer;
	i2cmXferRec.rxBuff 		= NULL;
	Chip_I2CM_XferBlocking( LPC_I2C0, &i2cmXferRec );
	res = SUCCESS;

	return res;
}

/**
* @brief  Reads a block of data from the MPU6050.
* @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
* @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
* @param  readAddr : MPU6050's internal address to read from.
* @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
* @return None
*/


Status qI2C_Read ( uint8_t slaveAddr, uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead )
{
	Status res;
//	I2C_M_SETUP_Type rxsetup;
//	rxsetup.sl_addr7bit = slaveAddr >> 1;
//	rxsetup.tx_data = &readAddr;
//	rxsetup.tx_length = 1;
//	rxsetup.rx_data = pBuffer;
//	rxsetup.rx_length = NumByteToRead;
//	rxsetup.retransmissions_max = MAX_RETRANSMISSION;
//	res = I2C_MasterTransferData(LPC_I2C1, &rxsetup, I2C_TRANSFER_POLLING);

	i2cmXferRec.slaveAddr 	= slaveAddr >> 1;
	i2cmXferRec.options 	= 0;
	i2cmXferRec.status 		= 0;
	i2cmXferRec.txSz 		= 1;
	i2cmXferRec.rxSz 		= NumByteToRead;
	i2cmXferRec.txBuff 		= &readAddr;
	i2cmXferRec.rxBuff 		= pBuffer;
	Chip_I2CM_XferBlocking( LPC_I2C0, &i2cmXferRec );
	res = SUCCESS;

	return res;
}

// 16bit addressing on device

Status qI2C_Write_ ( uint8_t slaveAddr,uint8_t* pBuffer, uint16_t writeAddr, uint16_t NumByteToWrite )
{
	uint8_t sendBuffer[NumByteToWrite+2];
	Status res;

	sendBuffer[0] = (uint8_t)(writeAddr & 0xFF00)>>8;
	sendBuffer[1] = (uint8_t)(writeAddr & 0x00FF);
	memcpy( &sendBuffer[2], pBuffer, NumByteToWrite );

//	I2C_M_SETUP_Type txsetup;
//	txsetup.sl_addr7bit = slaveAddr>>1;
//	txsetup.tx_data = sendBuffer;
//	txsetup.tx_length = sizeof(sendBuffer);
//	txsetup.rx_data = NULL;
//	txsetup.rx_length = 0;
//	txsetup.retransmissions_max = 3;
//	res = I2C_MasterTransferData(LPC_I2C1, &txsetup, I2C_TRANSFER_POLLING);

	i2cmXferRec.slaveAddr = slaveAddr >> 1;
	i2cmXferRec.options = 0;
	i2cmXferRec.status = 0;
	i2cmXferRec.txSz = sizeof(sendBuffer);
	i2cmXferRec.rxSz = 0;
	i2cmXferRec.txBuff = sendBuffer;
	i2cmXferRec.rxBuff = NULL;
	Chip_I2CM_XferBlocking( LPC_I2C0, &i2cmXferRec );
	res = SUCCESS;

	return res;
}

/**
* @brief  Reads a block of data from the MPU6050.
* @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
* @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
* @param  readAddr : MPU6050's internal address to read from.
* @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
* @return None
*/


Status qI2C_Read_(uint8_t slaveAddr, uint8_t* pBuffer, uint16_t readAddr, uint16_t NumByteToRead)
{
	Status res;
	uint8_t readAddress[2];
	//Tricky reading 16 bit addresses
	readAddress[0] = (uint8_t)(readAddr & 0xFF00)>>8;
	readAddress[1] = (uint8_t)(readAddr & 0x00FF);

//	I2C_M_SETUP_Type rxsetup;
//	rxsetup.sl_addr7bit = slaveAddr >> 1;
//	rxsetup.tx_data = readAddress;
//	rxsetup.tx_length = 2;
//	rxsetup.rx_data = pBuffer;
//	rxsetup.rx_length = NumByteToRead;
//	rxsetup.retransmissions_max = MAX_RETRANSMISSION;
//	res = I2C_MasterTransferData(LPC_I2C1, &rxsetup, I2C_TRANSFER_POLLING);

	i2cmXferRec.slaveAddr = slaveAddr >> 1;
	i2cmXferRec.options = 0;
	i2cmXferRec.status = 0;
	i2cmXferRec.txSz = 2;
	i2cmXferRec.rxSz = NumByteToRead;
	i2cmXferRec.txBuff = readAddress;
	i2cmXferRec.rxBuff = pBuffer;
	Chip_I2CM_XferBlocking( LPC_I2C0, &i2cmXferRec );
	res = SUCCESS;

	return res;
}

Status qI2C_WriteBit ( uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data )
{
	uint8_t tmp;
	Status res;
	res = qI2C_Read(slaveAddr,&tmp,regAddr,1);
	if ( res == SUCCESS )
	{
		tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
		res = qI2C_Write( slaveAddr, &tmp, regAddr, 1 );
	}
	return res;
}

Status qI2C_ReadBit ( uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t* data )
{
	uint8_t tmp;
	Status res;
	res = qI2C_Read( slaveAddr, &tmp, regAddr, 1 );
	*data = tmp & (1 << bitNum);
	return res;
}

Status qI2C_WriteBits ( uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data )
{
	uint8_t tmp;
	Status res;
	res = qI2C_Read( slaveAddr, &tmp, regAddr, 1 );
	if ( res == SUCCESS )
	{
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		tmp &= ~(mask); // zero all important bits in existing byte
		tmp |= data; // combine data with existing byte
		res = qI2C_Write(slaveAddr,&tmp,regAddr,1);
	}
	return res;
}

Status qI2C_ReadBits ( uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data )
{
	uint8_t tmp;
	Status res;
	res = qI2C_Read(slaveAddr,&tmp,regAddr,1);
	if ( res == SUCCESS )
	{
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		tmp &= mask;
		tmp >>= (bitStart - length + 1);
		*data = tmp;
	}
	return res;
}
