#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64];

// Orientation/motion variables
Quaternion q;
VectorFloat gravity;
float euler[3];
float ypr[3];
double yprd[3];

//  Use the following global variables 
//  to calibrate the gyroscope sensor and accelerometer readings
float    base_x_gyro = 0;
float    base_y_gyro = 0;
float    base_z_gyro = 0;
float    base_x_accel = 0;
float    base_y_accel = 0;
float    base_z_accel = 0;


// This global variable tells us how to scale gyroscope data
float    GYRO_FACTOR;

// This global varible tells how to scale acclerometer data
float    ACCEL_FACTOR;

// Buffer for data output
char dataOut[256];

double nickOut, nickSet = 0;
PID nickController(&yprd[2], &nickOut, &nickSet, 7, 0.0, 1.1, REVERSE);

double rollOut, rollSet = 0;
PID rollController(&yprd[1], &rollOut, &rollSet, 10.0, 0.06, 1.5, REVERSE);

double yawOut, yawSet = 0;
PID yawController(&yprd[0], &yawOut, &yawSet, 2.0, 0.0, 0.0, DIRECT);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;    

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}


//motor positions not yet messured!!
int northWestPin = 6;
int northEastPin = 5;
int southEastPin = 9;
int southWestPin = 11;

double zeroThrottle = 150;
double currentThrottle = 150;

void setup() {	
	Serial.begin(115200);

	//phase correct pwm on timer 1
	TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00); 
	//prescale factor of 64: 490 = 16000000 / (64 * 510)
	TCCR0B = _BV(CS00) | _BV(CS01);
	//do not forget to edit wiring.c to fix delay and milis

	// join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(1, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // Set the full scale range of the gyro
        uint8_t FS_SEL = 0;
        //mpu.setFullScaleGyroRange(FS_SEL);

        // get default full scale value of gyro - may have changed from default
        // function call returns values between 0 and 3
        uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
        Serial.print("FS_SEL = ");
        Serial.println(READ_FS_SEL);
        GYRO_FACTOR = 131.0/(FS_SEL + 1);
        

        // get default full scale value of accelerometer - may not be default value.  
        // Accelerometer scale factor doesn't reall matter as it divides out
        uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
        Serial.print("AFS_SEL = ");
        Serial.println(READ_AFS_SEL);
        //ACCEL_FACTOR = 16384.0/(AFS_SEL + 1);
        
        // Set the full scale range of the accelerometer
        //uint8_t AFS_SEL = 0;
        //mpu.setFullScaleAccelRange(AFS_SEL);

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

	//lowest pwm
	analogWrite(northWestPin, zeroThrottle);
	analogWrite(northEastPin, zeroThrottle);
	analogWrite(southEastPin, zeroThrottle);
	analogWrite(southWestPin, zeroThrottle);

	// configure LED for output
    pinMode(LED_PIN, OUTPUT);
	
	nickController.SetMode(AUTOMATIC);
	nickController.SetOutputLimits(-1000, 1000);
	rollController.SetMode(AUTOMATIC);
	rollController.SetOutputLimits(-1000, 1000);
	yawController.SetMode(AUTOMATIC);
	yawController.SetOutputLimits(-300, 300);

	while(true){
		Serial.print(".");
		if(Serial.available()){ // Wait for initialization command from user
			char command = Serial.read();
			if(command == 'X') {
				Serial.println("Starting copter");
				currentThrottle = 156;
				analogWrite(northWestPin, currentThrottle);
				analogWrite(northEastPin, currentThrottle);
				analogWrite(southEastPin, currentThrottle);
				analogWrite(southWestPin, currentThrottle);
				break;
			}
		}
		delay(500);
	}
}

void loop() {
	const float RADIANS_TO_DEGREES = 57.2958; //180/3.14159
 
    // if programming failed, don't try to do anything
    if (!dmpReady) return;   

    if(Serial.available()){
		char command = Serial.read();

		if(command == 'X') {
	        Serial.println("Stop");
	        currentThrottle = zeroThrottle;
		}

		if(command == 'F') {        
	        currentThrottle = currentThrottle + 1;
			Serial.print("Faster: ");
			Serial.println(currentThrottle);
		}	

		if(command == 'S') {        
	        currentThrottle = currentThrottle - 1;
			Serial.print("Slower: ");
			Serial.println(currentThrottle);
		}	

		if(command == 'E') {
	        Serial.println("Emergency Stop");
	        currentThrottle = 0;
		}

		analogWrite(northWestPin, currentThrottle);
		analogWrite(northEastPin, currentThrottle);
		analogWrite(southEastPin, currentThrottle);
		analogWrite(southWestPin, currentThrottle);
	}

    //Serial.println(currentThrottle);

	if(mpuInterrupt || fifoCount >= packetSize){
	    // reset interrupt flag and get INT_STATUS byte
	    mpuInterrupt = false;
	    mpuIntStatus = mpu.getIntStatus();

	    // get current FIFO count
	    fifoCount = mpu.getFIFOCount();

	    // check for overflow (this should never happen unless our code is too inefficient)
	    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	        // reset so we can continue cleanly
	        mpu.resetFIFO();
	        Serial.println(F("FIFO overflow!"));

	    // otherwise, check for DMP data ready interrupt (this should happen frequently)
	    } else if (mpuIntStatus & 0x02) {
		    // wait for correct available data length, should be a VERY short wait
		    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		    // read a packet from FIFO
		    mpu.getFIFOBytes(fifoBuffer, packetSize);
		    
		    // track FIFO count here in case there is > 1 packet available
		    // (this lets us immediately read more without waiting for an interrupt)
		    fifoCount -= packetSize;
		    
		    // Obtain Euler angles from buffer
		    //mpu.dmpGetQuaternion(&q, fifoBuffer);
		    //mpu.dmpGetEuler(euler, &q);
		    
		    // Obtain YPR angles from buffer
		    mpu.dmpGetQuaternion(&q, fifoBuffer);
		    mpu.dmpGetGravity(&gravity, &q);
		    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

		    yprd[0] = ypr[0];
		    yprd[1] = ypr[1];
		    yprd[2] = ypr[2];

		    nickController.Compute();
		    rollController.Compute();
		    yawController.Compute();
	  
			// Serial.print("DMP:");
			// Serial.print(yprd[2]*RADIANS_TO_DEGREES, 2);
			// Serial.print(":");
			// Serial.print(-yprd[1]*RADIANS_TO_DEGREES, 2);
			// Serial.print(":");
			// Serial.println(yprd[0]*RADIANS_TO_DEGREES, 2);   

			// Serial.print("PID:");   
			// Serial.print(nickOut, 2);
			// Serial.print(":");
			// Serial.print(rollOut, 2);
			// Serial.print(":");
			// Serial.println(yawOut, 2);

			double nw = currentThrottle - nickOut + rollOut - yawOut;
			double ne = currentThrottle - nickOut - rollOut + yawOut;
			double se = currentThrottle + nickOut - rollOut - yawOut;
			double sw = currentThrottle + nickOut + rollOut + yawOut;

			// analogWrite(northWestPin, nw);
			// analogWrite(northEastPin, ne);
			// analogWrite(southEastPin, se);
			// analogWrite(southWestPin, sw);

			Serial.print("PWM:");   
			Serial.print(nw, 2);
			Serial.print(":");
			Serial.print(ne, 2);
			Serial.print(":");
			Serial.print(se, 2);
			Serial.print(":");
			Serial.println(sw, 2);
		}
	}
}

