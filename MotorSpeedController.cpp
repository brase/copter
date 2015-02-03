#include "MotorSpeedController.h"

double yprd[3];
double nick, nickSet = 0;
PID nickController(&yprd[2], &nick, &nickSet, KP_NICK, KI_NICK, KD_NICK, REVERSE);

double roll, rollSet = 0;
PID rollController(&yprd[1], &roll, &rollSet, KP_ROLL, KI_ROLL, KD_ROLL, REVERSE);

double yaw, yawSet = 0;
PID yawController(&yprd[0], &yaw, &yawSet, KP_YAW, KI_YAW, KD_YAW, DIRECT);

void MotorSpeedController::init()
{
	nickController.SetMode(AUTOMATIC);
	nickController.SetOutputLimits(-127, 127);
	nickController.SetSampleTime(10);

	rollController.SetMode(AUTOMATIC);
	rollController.SetOutputLimits(-127, 127);
	rollController.SetSampleTime(10);

	yawController.SetMode(AUTOMATIC);
	yawController.SetOutputLimits(-127, 127);
	yawController.SetSampleTime(10);
}

void MotorSpeedController::compute(double currentThrottle, float ypr[3], const double wantedAngles[3], double motorThrottleValue[4])
{
	yprd[0] = ypr[0];
	yprd[1] = -ypr[1];
	yprd[2] = ypr[2];

	nickSet = wantedAngles[0];
	rollSet = wantedAngles[1];
	yawSet = wantedAngles[2];

	nickController.Compute();
	rollController.Compute();
	yawController.Compute();

	motorThrottleValue[0] = currentThrottle - nick + roll - yaw;
	motorThrottleValue[1] = currentThrottle - nick - roll + yaw;
	motorThrottleValue[2] = currentThrottle + nick - roll - yaw;
	motorThrottleValue[3] = currentThrottle + nick + roll + yaw;
}

