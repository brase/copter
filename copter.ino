// copter.ino

//motor positions not yet messured!!
int northWestPin = 9;
int northEastPin = 11;
int southEastPin = 3;
int southWestPin = 10;

int zeroThrottle = 150;
int currentThrottle = 150;

void setup() {	
	//lowest pwm
	analogWrite(northWestPin, zeroThrottle);
	analogWrite(northEastPin, zeroThrottle);
	analogWrite(southEastPin, zeroThrottle);
	analogWrite(southWestPin, zeroThrottle);

	Serial.begin(115200);
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

        Serial.println(currentThrottle);
	delay(200);
}

