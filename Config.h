#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <Arduino.h>

//#define STANGE

#define USB_MODE
#ifdef USB_MODE
	#define DEBUG Serial
#else
	#define DEBUG Serial1
#endif

#define SBUS_DEVICE Serial3

#define initialThrottle 150

#endif