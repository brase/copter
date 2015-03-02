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

double currentThrottle = 150;

void setup() {
	DEBUG.begin(115200);

	while (true){
		DEBUG.print(".");
		if (DEBUG.available()){ // Wait for initialization command from user
			char command = DEBUG.read();
			if (command == 'X') {
				DEBUG.println("Starting copter");
				break;
			}
		}
		delay(500);
	}

	rcReader.init();
	positionSensors.init();
	motorController.init();
	motors.init();

	motors.setSpeed(currentThrottle);
}

void loop() {
	int16_t pChannelData[7];
	rcReader.readRc(pChannelData);
	currentThrottle = pChannelData[2];

	if (Serial.available()){
		char command = Serial.read();

		if (command == 'X') {
			Serial.println("Stop");
			currentThrottle = motors.stop();
			USE_PID = false;
		}

		//if (command == 'F') {
		//	currentThrottle = currentThrottle + 1;
		//	Serial.print("Faster: ");
		//	Serial.println(currentThrottle);
		//}

		//if (command == 'S') {
		//	currentThrottle = currentThrottle - 1;
		//	Serial.print("Slower: ");
		//	Serial.println(currentThrottle);
		//}

		if (command == 'E') {
			Serial.println("Emergency Stop");
			currentThrottle = 100;
			USE_PID = false;
		}

		if (command == 'P'){
			Serial.println("Toggle PID");
			USE_PID = !USE_PID;
		}

		/*if (command == 'q'){
			double kp = rollController.GetKp() + 0.1;
			rollController.SetTunings(rollController.GetKp() + 0.1, rollController.GetKi(), rollController.GetKd());
			Serial.print("kp:   ");
			Serial.println(kp);
		}

		if (command == 'a'){
			double kp = rollController.GetKp() - 0.1;
			rollController.SetTunings(kp, rollController.GetKi(), rollController.GetKd());
			Serial.print("kp:   ");
			Serial.println(kp);
		}

		if (command == 'w'){
			double ki = rollController.GetKi() + 0.1;
			rollController.SetTunings(rollController.GetKp(), ki, rollController.GetKd());
			Serial.print("ki:   ");
			Serial.println(ki);
		}

		if (command == 's'){
			double ki = rollController.GetKi() - 0.1;
			rollController.SetTunings(rollController.GetKp(), ki, rollController.GetKd());
			Serial.print("ki:   ");
			Serial.println(ki);
		}

		if (command == 'e'){
			double kd = rollController.GetKd() + 0.1;
			rollController.SetTunings(rollController.GetKp(), rollController.GetKi(), kd);
			Serial.print("kd:   ");
			Serial.println(kd);
		}

		if (command == 'd'){
			double kd = rollController.GetKd() - 0.1;
			rollController.SetTunings(rollController.GetKp(), rollController.GetKi(), kd);
			Serial.print("kd:   ");
			Serial.println(kd);
		}*/
	}

	positionSensors.getYawPitchRoll(ypr);

	// DEBUG.print("DMP:");
	// DEBUG.print(yprd[2]*RADIANS_TO_DEGREES, 2);
	// DEBUG.print(":");
	// DEBUG.print(-yprd[1]*RADIANS_TO_DEGREES, 2);
	// DEBUG.print(":");
	// DEBUG.println(yprd[0]*RADIANS_TO_DEGREES, 2);   

	//	DEBUG.print("PID:");   
	//	DEBUG.print(nickOut, 2);
	//	DEBUG.print(":");
	//	DEBUG.print(rollOut, 2);
	//	DEBUG.print(":");
	//	DEBUG.println(yawOut, 2);

	if (USE_PID){
		double motorThrottleValues[4];
		double wantedAngles[3];
		wantedAngles[0] = 0;
		wantedAngles[1] = 0;
		wantedAngles[2] = 0;

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

	DEBUG.println(currentThrottle);

	// DEBUG.print("PWM:____");   
	// DEBUG.print(nw, 2);
	// DEBUG.print("____:____");
	// DEBUG.print(ne, 2);
	// DEBUG.print("____:____");
	// DEBUG.print(se, 2);
	// DEBUG.print("____:____");
	// DEBUG.println(sw, 2);
}