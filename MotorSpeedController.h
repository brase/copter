// MotorSpeedController.h
#include <Arduino.h>
#include <PID_v1.h>

#ifndef _MOTORSPEEDCONTROLLER_h
#define _MOTORSPEEDCONTROLLER_h

#define KP_NICK 0
#define KI_NICK 0
#define KD_NICK 0

#define KP_ROLL 4
#define KI_ROLL 0.4
#define KD_ROLL 1.8

#define KP_YAW 0
#define KI_YAW 0
#define KD_YAW 0

class MotorSpeedController
{
 protected:


 public:
	void init();
	void compute(double currentThrottle, float ypr[3], const double wantedAngles[3], double motorThrottleValue[4]);
private:

};

#endif

