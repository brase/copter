// PositionSensors.h

#ifndef _POSITIONSENSORS_h
#define _POSITIONSENSORS_h

#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"

class PositionSensors
{
 protected:


 public:
	void init();
	void getYawPitchRoll(float* ypr);
	bool dmpIsReady();
};

#endif

