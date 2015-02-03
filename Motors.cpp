#include "Motors.h"

void Motors::init()
{
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

//static void setPWMpin(uint32_t pin) {
//	PIO_Configure(g_APinDescription[pin].pPort,
//		PIO_PERIPH_B, //hack Arduino does not allow high PWM by default
//		g_APinDescription[pin].ulPin,
//		g_APinDescription[pin].ulPinConfiguration);
//}
//
//static void configOneMotor(uint8_t ch, uint32_t period) {
//	PWMC_ConfigureChannel(PWM, ch, PWM_CMR_CPRE_CLKA, 0, 0);
//	PWMC_SetPeriod(PWM, ch, period);
//	PWMC_SetDutyCycle(PWM, ch, MINCOMMAND);
//	PWMC_EnableChannel(PWM, ch);
//}
