#include "PositionSensors.h"
#include "MotorSpeedController.h"
#include "Motors.h"
#include <Wire.h>
#include "I2Cdev.h"
#include <PID_v1.h>

Motors motors;
MotorSpeedController motorController;
PositionSensors positionSensors;

float ypr[3];

bool USE_PID = false;

double currentThrottle = 150;

void setup() {
	Serial.begin(115200);

	////phase correct pwm on timer 1
	//TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
	////prescale factor of 64: 490 = 16000000 / (64 * 510)
	//TCCR0B = _BV(CS00) | _BV(CS01);
	////do not forget to edit wiring.c to fix delay and milis

	positionSensors.init();
	motorController.init();
	motors.init();

	while (true){
		Serial.print(".");
		if (Serial.available()){ // Wait for initialization command from user
			char command = Serial.read();
			if (command == 'X') {
				Serial.println("Starting copter");
				currentThrottle = 156;
				motors.setSpeed(currentThrottle);
				break;
			}
		}
		delay(500);
	}
}

void loop() {
	// if programming failed, don't try to do anything
	if (!positionSensors.dmpIsReady()) return;

	if (Serial.available()){
		char command = Serial.read();

		if (command == 'X') {
			Serial.println("Stop");
			currentThrottle = motors.stop();
			USE_PID = false;
		}

		if (command == 'F') {
			currentThrottle = currentThrottle + 1;
			Serial.print("Faster: ");
			Serial.println(currentThrottle);
		}

		if (command == 'S') {
			currentThrottle = currentThrottle - 1;
			Serial.print("Slower: ");
			Serial.println(currentThrottle);
		}

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

	//Serial.println(currentThrottle);

	positionSensors.getYawPitchRoll(ypr);

	// Serial.print("DMP:");
	// Serial.print(yprd[2]*RADIANS_TO_DEGREES, 2);
	// Serial.print(":");
	// Serial.print(-yprd[1]*RADIANS_TO_DEGREES, 2);
	// Serial.print(":");
	// Serial.println(yprd[0]*RADIANS_TO_DEGREES, 2);   

	//	Serial.print("PID:");   
	//	Serial.print(nickOut, 2);
	//	Serial.print(":");
	//	Serial.print(rollOut, 2);
	//	Serial.print(":");
	//	Serial.println(yawOut, 2);

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

	// Serial.print("PWM:____");   
	// Serial.print(nw, 2);
	// Serial.print("____:____");
	// Serial.print(ne, 2);
	// Serial.print("____:____");
	// Serial.print(se, 2);
	// Serial.print("____:____");
	// Serial.println(sw, 2);
}

