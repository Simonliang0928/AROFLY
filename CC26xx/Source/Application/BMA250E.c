#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/I2C.h>

#include "sensor.h"

#define I2C_BMA250E_ADDR	0x18

extern I2C_Handle I2cHandle;

void BMASensor_Write_U8(uint8_t add, uint8_t cmd)	//FMTI sensor write unsigned char function
{
	I2C_Transaction i2cTransaction;
	UChar writeBuffer[1];    

	i2cTransaction.writeBuf = writeBuffer; /* Buffer to be written */
	i2cTransaction.writeCount = 2; /* Number of bytes to be written */
	i2cTransaction.readBuf = NULL; /* Buffer to be read */
	i2cTransaction.readCount = 0; /* Number of bytes to be read */
  	i2cTransaction.slaveAddress = I2C_BMA250E_ADDR; /* 7-bit peripheral slave address */
 	writeBuffer[0] = add;
 	writeBuffer[1] = cmd;
	I2C_transfer(I2cHandle, &i2cTransaction); /* Perform I2C transfer */
}

uint8_t BMASensor_Read_U8(uint8_t add)						//FMTI sensor read unsigned char function
{
  	I2C_Transaction i2cTransaction;
	UChar readBuffer[1];
	UChar writeBuffer[1];    

	i2cTransaction.writeBuf = writeBuffer; /* Buffer to be written */
	i2cTransaction.writeCount = 1; /* Number of bytes to be written */
	i2cTransaction.readBuf = readBuffer; /* Buffer to be read */
	i2cTransaction.readCount = 1; /* Number of bytes to be read */
  	i2cTransaction.slaveAddress = I2C_BMA250E_ADDR; /* 7-bit peripheral slave address */
 	writeBuffer[0] = add;
	I2C_transfer(I2cHandle, &i2cTransaction); /* Perform I2C transfer */
	return (readBuffer[0]);
}



static uint8_t InitCompleted;
void BMA250E_init(){
	if(BMASensor_Read_U8(0x00)!=0xF9)
		InitCompleted = 0;
	else{
		InitCompleted = 1;
		// 31.25hz LP filter
		BMASensor_Write_U8(0x10, 0x0A);
		//+-16G range 
		BMASensor_Write_U8(0x0F, 0x0C);
	}	
}

void BMASensor_get(BMA250E_ACC_TMP *ptr)										//FMTI sensor read ADC value function
{
	I2C_Transaction i2cTransaction;
	UChar readBuffer[8];
	UChar writeBuffer[1];    

	if(!InitCompleted){
		memset((char*)ptr,0xFF, sizeof(BMA250E_ACC_TMP));
		return;
	}	
	i2cTransaction.writeBuf = writeBuffer; /* Buffer to be written */
	i2cTransaction.writeCount = 1; /* Number of bytes to be written */
	i2cTransaction.readBuf = readBuffer; /* Buffer to be read */
	i2cTransaction.readCount = 7; /* Number of bytes to be read */
  	i2cTransaction.slaveAddress = I2C_BMA250E_ADDR; /* 7-bit peripheral slave address */
 	writeBuffer[0] = 0x02;
	I2C_transfer(I2cHandle, &i2cTransaction); /* Perform I2C transfer */
	
	ptr->x = (readBuffer[0]>>6)|(readBuffer[1]<<2);
	if(ptr->x&0x0200) ptr->x|=0xFC00;
	ptr->y = (readBuffer[2]>>6)|(readBuffer[3]<<2);
	if(ptr->y&0x0200) ptr->y|=0xFC00;
	ptr->z = (readBuffer[4]>>6)|(readBuffer[5]<<2);
	if(ptr->z&0x0200) ptr->z|=0xFC00;
	ptr->t = readBuffer[6];	
	ptr->t += 23;	
}
