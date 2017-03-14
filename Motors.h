// Motors.h
#include "Config.h"
#include <Arduino.h>

#ifndef _MOTORS_h
#define _MOTORS_h

#define northWestPin 20
#define northEastPin 21
#define southEastPin 22
#define southWestPin 23



class Motors
{
 protected:


 public:
	void init();
	void setSpeed(uint32_t speed);
	void setSpeed(uint32_t nw, uint32_t ne, uint32_t se, uint32_t sw);
	uint32_t stop();
};

#endif

