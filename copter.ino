#include "Config.h"
#include "RC_Reader.h"
#include "PositionSensors.h"
#include "MotorSpeedController.h"
#include "Motors.h"
#include <Wire.h>
#include "I2Cdev.h"
#include <PID_v1.h>
#include "FUTABA_SBUS.h"
#include "RC_Reader.h"


Motors motors;
MotorSpeedController motorController;
PositionSensors positionSensors;
RC_Reader rcReader;

float ypr[3];

bool USE_PID = false;
bool USE_RC = false;

double currentThrottle = 150;

void setup() {
	DEBUG.begin(115200);

	rcReader.init();
	positionSensors.init();
	motorController.init();
	motors.init();

	USE_RC = true;
	USE_PID = true;
}

void loop() {
	if (DEBUG.available()){
		char command = DEBUG.read();

		if (command == 'X') {
			DEBUG.println("Stop");
			currentThrottle = motors.stop();
			USE_PID = false;
			USE_RC = false;
		}

		if (command == 'E') {
			DEBUG.println("Emergency Stop");
			currentThrottle = motors.stop();
			USE_RC = false;
			USE_PID = false;
		}

		if (command == 'P'){
			DEBUG.println("Toggle PID");
			USE_PID = !USE_PID;
		}

		/*if (command == 'q'){
			double kp = rollController.GetKp() + 0.1;
			rollController.SetTunings(rollController.GetKp() + 0.1, rollController.GetKi(), rollController.GetKd());
			DEBUG.print("kp:   ");
			DEBUG.println(kp);
		}

		if (command == 'a'){
			double kp = rollController.GetKp() - 0.1;
			rollController.SetTunings(kp, rollController.GetKi(), rollController.GetKd());
			DEBUG.print("kp:   ");
			DEBUG.println(kp);
		}

		if (command == 'w'){
			double ki = rollController.GetKi() + 0.1;
			rollController.SetTunings(rollController.GetKp(), ki, rollController.GetKd());
			DEBUG.print("ki:   ");
			DEBUG.println(ki);
		}

		if (command == 's'){
			double ki = rollController.GetKi() - 0.1;
			rollController.SetTunings(rollController.GetKp(), ki, rollController.GetKd());
			DEBUG.print("ki:   ");
			DEBUG.println(ki);
		}

		if (command == 'e'){
			double kd = rollController.GetKd() + 0.1;
			rollController.SetTunings(rollController.GetKp(), rollController.GetKi(), kd);
			DEBUG.print("kd:   ");
			DEBUG.println(kd);
		}

		if (command == 'd'){
			double kd = rollController.GetKd() - 0.1;
			rollController.SetTunings(rollController.GetKp(), rollController.GetKi(), kd);
			DEBUG.print("kd:   ");
			DEBUG.println(kd);
		}*/
	}

	double wantedAngles[3];
	wantedAngles[0] = 0;
	wantedAngles[1] = 0;
	wantedAngles[2] = 0;

	if (USE_RC){
		int16_t pChannelData[7];
		rcReader.readRc(pChannelData);

		//yaw
		wantedAngles[0] = ypr[0];
		//pitch
		wantedAngles[1] = (double)pChannelData[1] / 100;
		//roll
		wantedAngles[2] = (double)pChannelData[0] / 100;

		currentThrottle = (double)pChannelData[2];
	}

	positionSensors.getYawPitchRoll(ypr);

	//DEBUG.print("DMP:");
	//DEBUG.print(ypr[2], 2);
	//DEBUG.print(":");
	//DEBUG.print(-ypr[1], 2);
	//DEBUG.print(":");
	//DEBUG.println(ypr[0], 2);

	if (USE_PID){
		double motorThrottleValues[4];
		motorController.compute(currentThrottle, ypr, wantedAngles, motorThrottleValues);

		double nw = motorThrottleValues[0];
		double ne = motorThrottleValues[1];
		double se = motorThrottleValues[2];
		double sw = motorThrottleValues[3];

		motors.setSpeed(nw, ne, se, sw);
	}
	else {
		motors.setSpeed(currentThrottle);
	}
}