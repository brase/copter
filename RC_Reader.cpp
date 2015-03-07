// 
// 
// 
#include "RC_Reader.h"

void RC_Reader::init()
{
	sBus.begin();
	lastThrottle = 0;
	lastPitch = 0;
	lastRoll = 0;
}

 void RC_Reader::readRc(int16_t *pTarget){
	sBus.ProcessInput();

	if (sBus.IsDataAvailable())
	{
		int16_t pChannelData[7];
		uint8_t nStatusByte;

		sBus.FetchChannelData(pChannelData, nStatusByte);

		int roll = pChannelData[0];
		int pitch = pChannelData[1];
		int throttle = pChannelData[2];
		int yaw = pChannelData[3];

		if (nStatusByte == 0){
			//Read and normalize throttle
			if (throttle > 171 && throttle < 1812){
				lastThrottle = NormalizeThrottle(throttle);
			}
			lastRoll = NormalizeRoll(roll);
			lastPitch = NormalizePitch(pitch);
		}
	}

	pTarget[0] = lastRoll;
	pTarget[1] = lastPitch;
	pTarget[2] = lastThrottle;
	pTarget[3] = 0;
}

 float RC_Reader::NormalizeThrottle(int16_t iInput)
 {
	 uint16_t constrained = constrain(iInput, 172, 1812);
	 return map(constrained, 172, 1812, 151, 180);
 }

 float RC_Reader::NormalizePitch(int16_t iInput)
 {
	 uint16_t constrained = constrain(iInput, 172, 1811);
	 return map(constrained, 172, 1811, -100, 100);
 }

 float RC_Reader::NormalizeRoll(int16_t iInput)
 {
	 uint16_t constrained = constrain(iInput, 176, 1811);
	 return map(constrained, 176, 1811, -100, 100);
 }

 float RC_Reader::NormalizeYaw(int16_t iInput)
 {
	 uint16_t constrained = constrain(iInput, 172, 1811);
	 return map(constrained, 172, 1811, -100, 100);
 }