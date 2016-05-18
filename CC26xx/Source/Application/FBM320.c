#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/I2C.h>


#include "util.h"
#include "sensor.h"

#define I2C_FBM320_ADDR									0x6D
typedef struct FMTI_Sensor
{
	int32_t UP;
	int32_t UT;
	int32_t RP;
	uint16_t C0, C1, C2, C3, C6, C8, C9, C10, C11, C12; 
	uint32_t C4, C5, C7;
}FMTI_Sensor;

FMTI_Sensor FBM320;
extern I2C_Handle I2cHandle;

volatile char Trand;
void FMTISensor_Write_U8(uint8_t add, uint8_t cmd)	//FMTI sensor write unsigned char function
{
	I2C_Transaction i2cTransaction;
	UChar writeBuffer[1];    

	
	i2cTransaction.writeBuf = writeBuffer; /* Buffer to be written */
	i2cTransaction.writeCount = 2; /* Number of bytes to be written */
	i2cTransaction.readBuf = NULL; /* Buffer to be read */
	i2cTransaction.readCount = 0; /* Number of bytes to be read */
  	i2cTransaction.slaveAddress = I2C_FBM320_ADDR; /* 7-bit peripheral slave address */
 	writeBuffer[0] = add;
 	writeBuffer[1] = cmd;
	Trand = I2C_transfer(I2cHandle, &i2cTransaction); /* Perform I2C transfer */
}

uint8_t FMTISensor_Read_U8(uint8_t add)						//FMTI sensor read unsigned char function
{
  	I2C_Transaction i2cTransaction;
	UChar readBuffer[1];
	UChar writeBuffer[1];    

	i2cTransaction.writeBuf = writeBuffer; /* Buffer to be written */
	i2cTransaction.writeCount = 1; /* Number of bytes to be written */
	i2cTransaction.readBuf = readBuffer; /* Buffer to be read */
	i2cTransaction.readCount = 1; /* Number of bytes to be read */
  	i2cTransaction.slaveAddress = I2C_FBM320_ADDR; /* 7-bit peripheral slave address */
 	writeBuffer[0] = add;
	Trand = I2C_transfer(I2cHandle, &i2cTransaction); /* Perform I2C transfer */
	return (readBuffer[0]);
}
uint32_t FMTISensor_ReadADC_U32()										//FMTI sensor read ADC value function
{
	return	((uint32_t)FMTISensor_Read_U8(0xF6) << 16) | \
	        ((uint16_t)FMTISensor_Read_U8(0xF7) << 8) | \
	        FMTISensor_Read_U8(0xF8);
}
void FMTISensor_ReadCoefficient()									//FMTI sensor receive R0~R9 and calibrate coefficient C0~C12
{
	uint8_t i;
	uint16_t R[10]={0};
	
	for(i=0; i<9; i++)
		R[i] = ((uint8_t)FMTISensor_Read_U8(0xAA + (i*2)) << 8) | FMTISensor_Read_U8(0xAB + (i*2));
	R[9] = ((uint8_t)FMTISensor_Read_U8(0xD0) << 8) | FMTISensor_Read_U8(0xF1);
	
	
	/*	Use R0~R9 calculate C0~C12 of FBM320	*/
	FBM320.C0 = R[0] >> 4;
	FBM320.C1 = ((R[1] & 0xFF00) >> 5) | (R[2] & 7);
	FBM320.C2 = ((R[1] & 0xFF) << 1) | (R[4] & 1);
	FBM320.C3 = R[2] >> 3;
	FBM320.C4 = ((uint32_t)R[3] << 2) | (R[0] & 3);
	FBM320.C5 = R[4] >> 1;
	FBM320.C6 = R[5] >> 3;
	FBM320.C7 = ((uint32_t)R[6] << 3) | (R[5] & 7);
	FBM320.C8 = R[7] >> 3;
	FBM320.C9 = R[8] >> 2;
	FBM320.C10 = ((R[9] & 0xFF00) >> 6) | (R[8] & 3);
	FBM320.C11 = R[9] & 0xFF;
	FBM320.C12 = ((R[0] & 0x0C) << 1) | (R[7] & 7);
	

}
void FMTISensor_Calculate(int32_t UP, int32_t UT)										//FMTI sensor calculate real pressure & temperautre
{
	int32_t DT, DT2, X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32, CF, PP1, PP2, PP3, PP4;
		
	DT = ((UT - 8388608) >> 4) + (FBM320.C0 << 4);
	X01 = (FBM320.C1 + 4459) * DT >> 1;
	X02 = ((((FBM320.C2 - 256) * DT) >> 14) * DT) >> 4;
	X03 = (((((FBM320.C3 * DT) >> 18) * DT) >> 18) * DT);		
	DT2 = (X01 + X02 + X03) >> 12;
				
	X11 = ((FBM320.C5 - 21119) * DT2);
	X12 = ((((FBM320.C6 - 4574) * DT2) >> 16) * DT2) >> 1;
	X13 = ((FBM320.C4 - 23960) << 4) - ((X11 + X12) >> 13);
				
	X21 = ((FBM320.C8 + 18450) * DT2) >> 12;
	X22 = (((FBM320.C9 * DT2) >> 17) * DT2) >> 12;
	X23 = abs(X22 - X21);
	X24 = (X23 >> 11) * (FBM320.C7 + 551659);
	X25 = ((X23 & 0x7FF) * (FBM320.C7 + 551659)) >> 11;
	X26 = (X21 >= X22) ? (((0 - X24 - X25) >> 10) + FBM320.C7 + 551659) >> 3 : (((X24 + X25) >> 10) + FBM320.C7 + 551659) >> 3;

	PP1 = ((UP - 8388608) - X13) >> 3;
	PP2 = (X26 >> 10) * PP1;
	PP3 = ((X26 & 0x3FF) * PP1) >> 10;
	PP4 = (PP2 + PP3) >> 4;
				
	CF = (2097152 + FBM320.C12 * DT2) >> 3;
	X31 = (((CF * FBM320.C10) >> 21) * PP4) >> 2;
	X32 = (((((CF * FBM320.C11) >> 19) * PP4) >> 22) * PP4);
	FBM320.RP = ((X32 - X31) >> 11) + PP4 + 105000;

}
/*********************************************************************/

/* Temperature Clock instances for internal periodic events. */
static Clock_Struct Temp_measClock;
/* Pressure Clock instances for internal periodic events. */
static Clock_Struct Pres_measClock;

static void Measure_Handle(UArg arg)
{ 
	extern void Sensor_clockHandler(UArg arg);
	switch(arg){
		case TEMPERATURE_PERIODIC_EVT:
			Sensor_clockHandler(arg);
  			break;
  		case PRESSURE_PERIODIC_EVT:	
			/* Don to do anything */
  			break;
  	}
}

static uint8_t InitCompleted=0;
void FBM320_init(){
	if(FMTISensor_Read_U8(0x6B)!=0x42){
		InitCompleted = 0;
		return;
	}else{
		InitCompleted = 1;
		FMTISensor_ReadCoefficient();	
	}
	/* Create one-shot clocks for internal periodic events. */
  	Util_constructClock(&Temp_measClock, Measure_Handle,
                      TEMPERATURE_PERIOD, 0, false,
                      TEMPERATURE_PERIODIC_EVT);
    /* Create one-shot clocks for internal periodic events. */
  	Util_constructClock(&Pres_measClock, Measure_Handle,
                      PRESSURE_PERIOD, 0, false,
                      PRESSURE_PERIODIC_EVT);	                  
}

//void FBM320_Meas_Trigger(){
//	if(!InitCompleted)
//		return(0xFFFFFFFF);
//	/* Convert temperature ADC */	
//	FMTISensor_Write_U8(0xF4, 0x2E);				
//	simon = FMTISensor_Read_U8(0xF4);
//	//delay 2.2ms
//	FBM320.UT = FMTISensor_ReadADC_U32();									//Receive temperature ADC value
//	FMTISensor_Write_U8(0xF4, 0xF4);										//Convert pressure ADC
//	simon = FMTISensor_Read_U8(0xF4);
//	//delay 9.8ms
//	FBM320.UP = FMTISensor_ReadADC_U32();									//Receive pressure ADC value
//	FMTISensor_Calculate(FBM320.UP, FBM320.UT);
//	return(FBM320.RP);
//}		

uint32_t FBMSensor_get(){
	if(!InitCompleted){
		FBM320.RP = 0xFFFFFFFF;
	}else{	
		/* Receive pressure ADC value */
  		FBM320.UP = FMTISensor_ReadADC_U32();
  		/* Convert temperature ADC */	
		FMTISensor_Write_U8(0xF4, 0x2E);
		/* start pressure clock */
  		Util_startClock(&Temp_measClock);
  		/* Calculate barometer */ 
  		FMTISensor_Calculate(FBM320.UP, FBM320.UT);
  	}
  	/* return old value */
	return(FBM320.RP);
}
	
void FBMSensor_Start(){
	/* Receive temperature ADC value */
  	FBM320.UT = FMTISensor_ReadADC_U32();
  	/* start to Convert pressure ADC */
  	FMTISensor_Write_U8(0xF4, 0xF4);			
  	/* start pressure clock */
  	Util_startClock(&Pres_measClock);	
}
