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
	lastYaw = 0;
	lastOn = 0;
}

void RC_Reader::readRc(int16_t *pTarget){
	sBus.ProcessInput();

	if (sBus.IsDataAvailable())
	{
		int16_t pChannelData[7];
		uint8_t nStatusByte;

		sBus.FetchChannelData(pChannelData, nStatusByte);

		if (nStatusByte == 0){
			int roll = pChannelData[0];
			int pitch = pChannelData[1];
			int throttle = pChannelData[2];
			int yaw = pChannelData[3];
			int on = pChannelData[4];

			//Read and normalize throttle
			if (throttle > 171 && throttle < 1812){
				lastThrottle = NormalizeThrottle(throttle);
			}
			lastRoll = NormalizeRoll(roll);
			lastPitch = NormalizePitch(pitch);
			lastYaw = NormalizeYaw(yaw);
			DEBUG.print("On: ");
			DEBUG.println(on);

			lastOn = NormalizeOnOffSwitch(on);
		}
	}

	pTarget[0] = lastRoll;
	pTarget[1] = lastPitch;
	pTarget[2] = lastThrottle;
	pTarget[3] = lastYaw;
	pTarget[4] = lastOn;
}

 float RC_Reader::NormalizeThrottle(int16_t iInput)
 {
	 uint16_t constrained = constrain(iInput, 172, 1812);
	 return map(constrained, 172, 1812, 151, 220);
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

 float RC_Reader::NormalizeOnOffSwitch(int16_t iInput)
 {
	 uint16_t constrained = constrain(iInput, 172, 1811);
	 return map(constrained, 172, 1811, 0, 1);
 }