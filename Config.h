#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <Arduino.h>

//#define USB_MODE
#ifdef USB_MODE
	#define DEBUG Serial
#else
	#define DEBUG Serial1
#endif

#define SBUS_DEVICE Serial3

#endif