/*
library name: 	MPU6050 6 axis module
written by: 		T.Jaber
Date Written: 	25 Mar 2019
Last Modified: 	20 April 2019 by Mohamed Yaqoob
Description: 		MPU6050 Module Basic Functions Device Driver library that use HAL libraries.
References:			
								- MPU6050 Registers map: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
								- Jeff Rowberg MPU6050 library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
								
* Copyright (C) 2019 - T. Jaber
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.
	
   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.

*/

//Header files
#include "TJ_MPU6050.h"

//Library Variable
//1- I2C Handle 
static I2C_HandleTypeDef i2cHandler;
//2- Accel & Gyro Scaling Factor
static float accelScalingFactor, gyroScalingFactor;
//3- Bias variables
static float A_X_Bias = 0.0f;
static float A_Y_Bias = 0.0f;
static float A_Z_Bias = 0.0f;
//4- Gyro Bias variables
static float G_X_Bias = 0.0f;
static float G_Y_Bias = 0.0f;
static float G_Z_Bias = 0.0f;

static int16_t GyroRW[3];

//Function Definitions
//1- i2c Handler
void MPU6050_Init(I2C_HandleTypeDef *I2Chnd)
{
	//Copy I2C CubeMX handle to local library
	memcpy(&i2cHandler, I2Chnd, sizeof(*I2Chnd));
}

//2- i2c Read
HAL_StatusTypeDef I2C_Read(uint8_t ADDR, uint8_t *i2cBif, uint8_t NofData)
{
	uint8_t i2cBuf[2];
	uint8_t MPUADDR;
	HAL_StatusTypeDef status;
	//Need to Shift address to make it proper to i2c operation
	MPUADDR = (MPU_ADDR<<1);
	i2cBuf[0] = ADDR;
	status = HAL_I2C_Master_Transmit(&i2cHandler, MPUADDR, i2cBuf, 1, 10);
	if(status != HAL_OK)
		return status;
	return HAL_I2C_Master_Receive(&i2cHandler, MPUADDR, i2cBif, NofData, 100);
}

//3- i2c Write
HAL_StatusTypeDef I2C_Write8(uint8_t ADDR, uint8_t data)
{
	uint8_t i2cData[2];
	i2cData[0] = ADDR;
	i2cData[1] = data;
	uint8_t MPUADDR = (MPU_ADDR<<1);
	return HAL_I2C_Master_Transmit(&i2cHandler, MPUADDR, i2cData, 2, 100);
}

//4- MPU6050 Initialization Configuration 
void MPU6050_Config(MPU_ConfigTypeDef *config)
{
	uint8_t Buffer = 0;
	//Clock Source 
	//Reset Device
	I2C_Write8(PWR_MAGT_1_REG, 0x80);
	HAL_Delay(100);
	Buffer = config ->ClockSource & 0x07; //change the 7th bits of register
	Buffer |= (config ->Sleep_Mode_Bit << 6) &0x40; // change only the 7th bit in the register
	I2C_Write8(PWR_MAGT_1_REG, Buffer);
	HAL_Delay(100); // should wait 10ms after changeing the clock setting.
	
	//Set the Digital Low Pass Filter
	Buffer = 0;
	Buffer = config->CONFIG_DLPF & 0x07;
	I2C_Write8(CONFIG_REG, Buffer);
	
	//Select the Gyroscope Full Scale Range
	Buffer = 0;
	Buffer = (config->Gyro_Full_Scale << 3) & 0x18;
	I2C_Write8(GYRO_CONFIG_REG, Buffer);
	
	//Select the Accelerometer Full Scale Range 
	Buffer = 0; 
	Buffer = (config->Accel_Full_Scale << 3) & 0x18;
	I2C_Write8(ACCEL_CONFIG_REG, Buffer);
	//Set SRD To Default
	MPU6050_Set_SMPRT_DIV(0x04);
	
	
	//Accelerometer Scaling Factor, Set the Accelerometer and Gyroscope Scaling Factor
	switch (config->Accel_Full_Scale)
	{
		case AFS_SEL_2g:
			accelScalingFactor = (2000.0f/32768.0f);
			break;
		
		case AFS_SEL_4g:
			accelScalingFactor = (4000.0f/32768.0f);
				break;
		
		case AFS_SEL_8g:
			accelScalingFactor = (8000.0f/32768.0f);
			break;
		
		case AFS_SEL_16g:
			accelScalingFactor = (16000.0f/32768.0f);
			break;
		
		default:
			break;
	}
	//Gyroscope Scaling Factor 
	switch (config->Gyro_Full_Scale)
	{
		case FS_SEL_250:
			gyroScalingFactor = 250.0f/32768.0f;
			break;
		
		case FS_SEL_500:
				gyroScalingFactor = 500.0f/32768.0f;
				break;
		
		case FS_SEL_1000:
			gyroScalingFactor = 1000.0f/32768.0f;
			break;
		
		case FS_SEL_2000:
			gyroScalingFactor = 2000.0f/32768.0f;
			break;
		
		default:
			break;
	}
	
}

//5- Get Sample Rate Divider
uint8_t MPU6050_Get_SMPRT_DIV(void)
{
	uint8_t Buffer = 0;
	
	I2C_Read(SMPLRT_DIV_REG, &Buffer, 1);
	return Buffer;
}

//6- Set Sample Rate Divider
void MPU6050_Set_SMPRT_DIV(uint8_t SMPRTvalue)
{
	I2C_Write8(SMPLRT_DIV_REG, SMPRTvalue);
}

//7- Get External Frame Sync.
uint8_t MPU6050_Get_FSYNC(void)
{
	uint8_t Buffer = 0;
	
	I2C_Read(CONFIG_REG, &Buffer, 1);
	Buffer &= 0x38; 
	return (Buffer>>3);
}

//8- Set External Frame Sync. 
void MPU6050_Set_FSYNC(enum EXT_SYNC_SET_ENUM ext_Sync)
{
	uint8_t Buffer = 0;
	I2C_Read(CONFIG_REG, &Buffer,1);
	Buffer &= ~0x38;
	
	Buffer |= (ext_Sync <<3); 
	I2C_Write8(CONFIG_REG, Buffer);
	
}

//9- Get Accel Raw Data
void MPU6050_Get_Accel_RawData(RawData_Def *rawDef)
{
	uint8_t i2cBuf[2];
	uint8_t AcceArr[6], GyroArr[6];
	
	I2C_Read(INT_STATUS_REG, &i2cBuf[1],1);
	if((i2cBuf[1] & 0x01))
	{
		I2C_Read(ACCEL_XOUT_H_REG, AcceArr,6);
		
		//Accel Raw Data
		rawDef->x = ((AcceArr[0]<<8) + AcceArr[1]); // x-Axis
		rawDef->y = ((AcceArr[2]<<8) + AcceArr[3]); // y-Axis
		rawDef->z = ((AcceArr[4]<<8) + AcceArr[5]); // z-Axis
		//Gyro Raw Data
		I2C_Read(GYRO_XOUT_H_REG, GyroArr,6);
		GyroRW[0] = ((GyroArr[0]<<8) + GyroArr[1]);
		GyroRW[1] = (GyroArr[2]<<8) + GyroArr[3];
		GyroRW[2] = ((GyroArr[4]<<8) + GyroArr[5]);
	}
}

//10- Get Accel scaled data (g unit of gravity, 1g = 9.81m/s2)
void MPU6050_Get_Accel_Scale(ScaledData_Def *scaledDef)
{

	RawData_Def AccelRData;
	MPU6050_Get_Accel_RawData(&AccelRData);
	
	//Accel Scale data 
	scaledDef->x = ((AccelRData.x+0.0f)*accelScalingFactor);
	scaledDef->y = ((AccelRData.y+0.0f)*accelScalingFactor);
	scaledDef->z = ((AccelRData.z+0.0f)*accelScalingFactor);
}

//11- Get Accel calibrated data
void MPU6050_Get_Accel_Cali(ScaledData_Def *CaliDef)
{
	ScaledData_Def AccelScaled;
	MPU6050_Get_Accel_Scale(&AccelScaled);
	
	//Accel Scale data 
	CaliDef->x = (AccelScaled.x) - A_X_Bias; // x-Axis
	CaliDef->y = (AccelScaled.y) - A_Y_Bias;// y-Axis
	CaliDef->z = (AccelScaled.z) - A_Z_Bias;// z-Axis
}
//12- Get Gyro Raw Data
void MPU6050_Get_Gyro_RawData(RawData_Def *rawDef)
{
	
	//Accel Raw Data
	rawDef->x = GyroRW[0];
	rawDef->y = GyroRW[1];
	rawDef->z = GyroRW[2];
	
}

//13- Get Gyro scaled data
void MPU6050_Get_Gyro_Scale(ScaledData_Def *scaledDef)
{
	RawData_Def myGyroRaw;
	MPU6050_Get_Gyro_RawData(&myGyroRaw);
	
	//Gyro Scale data 
	scaledDef->x = (myGyroRaw.x)*gyroScalingFactor; // x-Axis
	scaledDef->y = (myGyroRaw.y)*gyroScalingFactor; // y-Axis
	scaledDef->z = (myGyroRaw.z)*gyroScalingFactor; // z-Axis
}

//14- Accel Calibration
void _Accel_Cali(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
{
	//1* X-Axis calibrate
	A_X_Bias		= (x_max + x_min)/2.0f;
	
	//2* Y-Axis calibrate
	A_Y_Bias		= (y_max + y_min)/2.0f;
	
	//3* Z-Axis calibrate
	A_Z_Bias		= (z_max + z_min)/2.0f;
}

//15- Gyro Calibration
//    Call this function while the sensor is stationary.
//    numSamples: number of samples to average for bias estimation (min 10, recommended 500)
//    Note: HAL_Delay(1) between samples ensures fresh readings; ~1ms per sample execution time.
//    Returns HAL_OK on success, HAL_ERROR if no valid I2C samples were collected
HAL_StatusTypeDef _Gyro_Cali(uint16_t numSamples)
{
	float rawSumX = 0.0f, rawSumY = 0.0f, rawSumZ = 0.0f;
	uint8_t GyroArr[6];
	uint16_t validSamples = 0;
	
	for(uint16_t i = 0; i < numSamples; i++)
	{
		if(I2C_Read(GYRO_XOUT_H_REG, GyroArr, 6) == HAL_OK)
		{
			rawSumX += (int16_t)((GyroArr[0]<<8) | GyroArr[1]);
			rawSumY += (int16_t)((GyroArr[2]<<8) | GyroArr[3]);
			rawSumZ += (int16_t)((GyroArr[4]<<8) | GyroArr[5]);
			validSamples++;
		}
		HAL_Delay(1);
	}
	
	if(validSamples == 0)
		return HAL_ERROR;
	
	// Average the raw counts and convert to deg/s for direct subtraction in MPU6050_Get_Gyro_Cali
	G_X_Bias = (rawSumX / (float)validSamples) * gyroScalingFactor;
	G_Y_Bias = (rawSumY / (float)validSamples) * gyroScalingFactor;
	G_Z_Bias = (rawSumZ / (float)validSamples) * gyroScalingFactor;
	return HAL_OK;
}

//16- Get Gyro calibrated data
void MPU6050_Get_Gyro_Cali(ScaledData_Def *CaliDef)
{
	ScaledData_Def GyroScaled;
	MPU6050_Get_Gyro_Scale(&GyroScaled);
	
	CaliDef->x = GyroScaled.x - G_X_Bias; // x-Axis
	CaliDef->y = GyroScaled.y - G_Y_Bias; // y-Axis
	CaliDef->z = GyroScaled.z - G_Z_Bias; // z-Axis
}

//17- Read WHO_AM_I register for device verification
//    Returns 0x68 if the correct MPU6050 device is connected, 0x00 on I2C error
uint8_t MPU6050_ReadID(void)
{
	uint8_t Buffer = 0;
	if(I2C_Read(WHO_AM_I_REG, &Buffer, 1) != HAL_OK)
		return 0x00;
	return Buffer;
}

//18- Get internal temperature sensor reading (degrees Celsius)
//    Formula from MPU6050 datasheet: Temp[°C] = TEMP_OUT / 340 + 36.53
//    Returns HAL_StatusTypeDef; *Temperature is only valid when HAL_OK is returned
HAL_StatusTypeDef MPU6050_Get_Temperature(float *Temperature)
{
	uint8_t TempArr[2];
	int16_t tempRaw;
	HAL_StatusTypeDef status;
	
	status = I2C_Read(TEMP_OUT_H_REG, TempArr, 2);
	if(status != HAL_OK)
		return status;
	tempRaw = (int16_t)((TempArr[0]<<8) | TempArr[1]);
	*Temperature = (tempRaw / 340.0f) + 36.53f;
	return HAL_OK;
}
