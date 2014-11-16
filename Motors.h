// Motors.h

#ifndef _MOTORS_h
#define _MOTORS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#define KP_NICK 0
#define KI_NICK 0
#define KD_NICK 0

#define KP_ROLL 4
#define KI_ROLL 0.4
#define KD_ROLL 1.8

#define KP_YAW 0
#define KI_YAW 0
#define KD_YAW 0

#define northWestPin 11
#define northEastPin 10
#define southEastPin 9
#define southWestPin 6

class MotorsClass
{
 protected:


 public:
	void init();
};

extern MotorsClass Motors;

#endif

