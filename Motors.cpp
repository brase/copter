#include "Motors.h"

void Motors::init()
{
	pinMode(northWestPin, OUTPUT);
	pinMode(northEastPin, OUTPUT);
	pinMode(southEastPin, OUTPUT);
	pinMode(southWestPin, OUTPUT);

	stop();
}

void Motors::setSpeed(uint32_t speed)
{
	setSpeed(speed, speed, speed, speed);
}

void Motors::setSpeed(uint32_t nw, uint32_t ne, uint32_t se, uint32_t sw)
{
	analogWrite(northWestPin, nw);
	analogWrite(northEastPin, ne);
	analogWrite(southEastPin, se);
	analogWrite(southWestPin, sw);
}

uint32_t Motors::stop()
{
	setSpeed(initialThrottle, initialThrottle, initialThrottle, initialThrottle);

	return initialThrottle;
}