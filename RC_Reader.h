// RC_Reader.h

#ifndef _RC_READER_h
#define _RC_READER_h

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

	float NormalizeThrottle(int16_t iInput);
};

#endif

