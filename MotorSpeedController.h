// MotorSpeedController.h
#include "Config.h"
#include <Arduino.h>
#include <PID_v1.h>

#ifndef _MOTORSPEEDCONTROLLER_h
#define _MOTORSPEEDCONTROLLER_h

#ifdef STANGE
#define KP_NICK 0
#define KI_NICK 0
#define KD_NICK 0
#else
#define KP_NICK 5.5
#define KI_NICK 0
#define KD_NICK 0
#endif

#define KP_ROLL 5.5
#define KI_ROLL 0
#define KD_ROLL 0

#define KP_YAW 5.5
#define KI_YAW 0
#define KD_YAW 0

class MotorSpeedController
{
 protected:


 public:
	void init();
	void compute(double currentThrottle, float ypr[3], const double wantedAngles[3], double motorThrottleValue[4]);
	void reset();
private:

};

#endif

