
#include "magnetometr.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

#define sizeOfBufferForRawMagnitude 100

OrientationInSpace rawMagnitudeBuffer[sizeOfBufferForRawMagnitude];
uint16_t rawMagnitudeBufferIndex = 0;

void initHMC5883L()
{
	uint8_t array[2];
	array[0] = 1;
	array[1] = 0x1D;

	HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADRESS, 0x0B, 1, &array[0], 1, 100);
	HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADRESS, 0x09, 1, &array[1], 1, 100);
}

void waitTillMagnetometerIsInitialized()
{
	while (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)HMC5883L_ADRESS, 10, 100) != HAL_OK)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_Delay(500);
	}
}

uint8_t checkAvalibilityOfDataInRegister()
{
	uint8_t someOutput[6];
	HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADRESS, 0x06, 1, someOutput, 6, 100);

	if((someOutput[0] & 0x01) == 1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

float calculateAzimutWithDegree()
{
	OrientationInSpace orientationInSpace = readRawDataFromMagnetometer();

	return atan2f((float)orientationInSpace.yAxis,
				  (float)orientationInSpace.xAxis)*(180.0f/M_PI);
}

OrientationInSpace readRawDataFromMagnetometer()
{

	OrientationInSpace rawOrientationInSpace;
	uint8_t bufferForMeasurement[6];

	HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADRESS, 0x03, 1, bufferForMeasurement, 6, 100);
	rawOrientationInSpace.xAxis = (bufferForMeasurement[1]<<8) | bufferForMeasurement[0];
	rawOrientationInSpace.yAxis = (bufferForMeasurement[5]<<8) | bufferForMeasurement[4];
	rawOrientationInSpace.zAxis = (bufferForMeasurement[3]<<8) | bufferForMeasurement[2];

	return rawOrientationInSpace;
}
