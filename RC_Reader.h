// RC_Reader.h

#ifndef _RC_READER_h
#define _RC_READER_h

#include "Config.h"
#include "Arduino.h"
#include "FUTABA_SBUS.h"

class RC_Reader
{
 protected:


 public:
	void init();
	void readRc(int16_t *pTarget);
 private:
	FUTABA_SBUS sBus;
	int16_t lastThrottle;
	int16_t lastPitch;
	int16_t lastRoll;

	float NormalizeThrottle(int16_t iInput);
	float NormalizeRoll(int16_t iInput);
	float NormalizePitch(int16_t iInput);
	float NormalizeYaw(int16_t iInput);
};

#endif

