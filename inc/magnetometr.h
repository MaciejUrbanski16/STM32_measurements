
#ifndef INC_MAGNETOMETR_H_
#define INC_MAGNETOMETR_H_

#include "stm32f4xx_hal.h"

#define HMC5883L_ADRESS 0x1A

#define configurationA  0x78
#define configurationB 0xA0
#define mode 0x02

#define XaxisMSB 0x03
#define XaxisLSB 0x04

#define YaxisMSB 0x07
#define YaxisLSB 0x08

#define ZaxisMSB 0x05
#define ZaxisLSB 0x06

#define status 0x09

typedef struct
{
	int16_t xAxis;
	int16_t yAxis;
	int16_t zAxis;

	float azimutInDegree;
}OrientationInSpace;

void initHMC5883L();

void waitTillMagnetometerIsInitialized();

uint8_t checkAvalibilityOfDataInRegister();

float calculateAzimutWithDegree();

OrientationInSpace readRawDataFromMagnetometer();

#endif /* INC_MAGNETOMETR_H_ */
