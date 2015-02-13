// Motors.h

#include <Arduino.h>

#ifndef _MOTORS_h
#define _MOTORS_h

#define northWestPin 11
#define northEastPin 10
#define southEastPin 9
#define southWestPin 6

#define initialThrottle 150

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
