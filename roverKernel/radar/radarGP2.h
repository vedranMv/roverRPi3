/**
 * radarGP2.h
 *
 *  Created on: 29. 5. 2016.
 *      Author: Vedran
 *
 *  IR-sensor based radar (on 2D gimbal)
 *  (library Infrared Proximity Sensor, Sharp GP2Y0A21YK)
 *  @version 1.2.1
 *  v1.1
 *  +Packed sensor functions and data into a C++ object
 *  V1.2
 *  +Added internal data buffer (dynamically allocated)
 *  +Object is now kernel module (added support for task scheduler)
 *  +Added hook to use function to execute when a scan is completed
 *  V1.2.1 - 6.2.2017
 *  +Modified to support Task scheduler v2.3
 *
 **** Hardware dependencies:
 *      PWM - Generator 1 & 3
 *      G1:PWM Out1(PF1 - horiz. axis), G3:PWM Out4(PG0 - vert. axis)
 *      ADC0: AIN3(PE0) - sampling sensor output
 *	IMPORTANT: PWM module runs with clock divider of 32, should it be changed to
 *		e.g. 64, all number passed to PWM have to be halved (i.e. RADAR_LEFT
 *		will be 26980, RADAR_RIGHT 29870 etc.)
 */

#ifndef RADARGP2_H_
#define RADARGP2_H_

//  Enable integration of this library with task scheduler
#define __USE_TASK_SCHEDULER__

#if defined(__USE_TASK_SCHEDULER__)
    #include "../taskScheduler/taskScheduler.h"
    //  Unique identifier of this module as registered in task scheduler
    #define RADAR_UID       1
    //  Definitions of ServiceID for service offered by this module
    #define RADAR_SCAN      0   //  Initiate radar scan
    #define RADAR_SETH      2   //  Set horizontal angle for radar
    #define RADAR_SETV      3   //  Set vertical angle of radar

#endif

class RadarModule
{
    friend void _RADAR_KernelCallback(void);
	public:
		RadarModule();
		~RadarModule();

		void InitHW();
		void AddHook(void((*funPoint)(uint8_t*, uint16_t*)));
		void Scan(uint8_t *data, uint16_t *length, bool fine);
		void Scan(bool fine);
		bool ScanReady();

		void SetHorAngle(float angle);
        void SetVerAngle(float angle);

        //  Hook to user routine called when the scan is complete
        void    ((*custHook)(uint8_t*, uint16_t*));

	protected:
		bool    _scanComplete;
		uint8_t *_scanData;
		bool    _fineScan;
#if defined(__USE_TASK_SCHEDULER__)
		_kernelEntry _radKer;
#endif
};

//  Global pointer to FIRST created instance of RadarModule
extern RadarModule* __rD;

#endif /* RADARGP2_H_ */

