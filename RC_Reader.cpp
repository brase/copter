// 
// 
// 

#include "RC_Reader.h"

void RC_Reader::init()
{
	sBus.begin();
	lastThrottle = 0;
}

 void RC_Reader::readRc(int16_t *pTarget){
	sBus.ProcessInput();

	if (sBus.IsDataAvailable())
	{
		int16_t pChannelData[7];
		uint8_t nStatusByte;

		sBus.FetchChannelData(pChannelData, nStatusByte);
		int throttle = pChannelData[2];

		if (nStatusByte == 0){
			//Read and normalize throttle
			if (throttle > 171 && throttle < 1812){
				lastThrottle = NormalizeThrottle(throttle);
			}
		}
	}

	pTarget[2] = lastThrottle;
}

 float RC_Reader::NormalizeThrottle(int16_t iInput)
 {
	 //return (iInput - 1024) / 2048.0f + 0.5f;

	 uint16_t constrained = constrain(iInput, 172, 1812);
	 return map(constrained, 172, 1811, 155, 180);
 }
