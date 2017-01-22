/*
 * radarGP2.h
 *
 *  Created on: 29. 5. 2016.
 *      Author: Vedran
 *
 **** Hardware dependencies:
 * 	PE0 - AIN3 - Reads analog value from sensor(@1MSps/64)
 * 		Hardware averaging of 64 samples
 * 	PF1 - PWMOut1 - Controls radar angle (PWM block 0, Generator 0, Output 1)
 * 		Gen0 with divider 32 generates ~60Hz PWM (62498 passed as argument)
 * 		PWMOut1 runs between 53960 and 59740
 *	IMPORTANT: PWM module runs with clock divder of 32, should it be changed to
 *		e.g. 64, all number passed to PWM have to be halved (i.e. RADAR_LEFT
 *		will be 26980, RADAR_RIGHT 29870 etc.)
 */

#ifndef RADARGP2_H_
#define RADARGP2_H_

class RadarData
{
	public:
		RadarData();
		~RadarData();

		int8_t InitHW();
		int8_t Scan(uint8_t *data, uint16_t *length, bool fine);
		bool ScanReady();

	protected:
		void _SetVerAngle(float angle);
		void _SetHorAngle(float angle);
		bool _scanComplete;
};

extern RadarData* __rD;

#endif /* RADARGP2_H_ */

