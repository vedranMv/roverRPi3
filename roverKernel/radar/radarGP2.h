/**
 * radarGP2.h
 *
 *  Created on: 29. 5. 2016.
 *      Author: Vedran
 *
 *  IR-sensor based radar (on 2D gimbal)
 *  (library Infrared Proximity Sensor, Sharp GP2Y0A21YK)
 *  @version 1.2.3
 *  v1.1
 *  +Packed sensor functions and data into a C++ object
 *  V1.2
 *  +Added internal data buffer (dynamically allocated)
 *  +Object is now kernel module (added support for task scheduler)
 *  +Added hook to use function to execute when a scan is completed
 *  V1.2.1 - 6.2.2017
 *  +Modified to support Task scheduler v2.3
 *  V1.2.2 - 9.3.2017
 *  +Changed RadarModule class into a singleton
 *  V1.2.3 - 2.7.2017
 *  +Change include paths for better portability, new way of printing to debug
 *  +Integration with event logger
 *  TODO: Remove fine scanning -> not needed
 *        Implement sweeping the radar through task scheduler, periodic task
 *        repeated 160 times, every 5 to 7ms
 */
#include "hwconfig.h"

//  Compile following section only if hwconfig.h says to include this module
#if !defined(RADARGP2_H_) && defined(__HAL_USE_RADAR__)
#define RADARGP2_H_

//  Enable integration of this library with task scheduler but only if task
//  scheduler is being compiled into this project
#if defined(__HAL_USE_TASKSCH__)
#define __USE_TASK_SCHEDULER__
#endif  /* __HAL_USE_TASKSCH__ */

//  Check if this library is set to use task scheduler
#if defined(__USE_TASK_SCHEDULER__)
    #include "taskScheduler/taskScheduler.h"
    //  Unique identifier of this module as registered in task scheduler
    #define RADAR_UID       1
    //  Definitions of ServiceID for service offered by this module
    #define RADAR_SCAN      0   //  Initiate radar scan
    #define RADAR_SETH      1   //  Set horizontal angle for radar
    #define RADAR_SETV      2   //  Set vertical angle of radar
    #define RADAR_SWEEPSTEP 3   //  Change of angle and measurement

#endif /* __USE_TASK_SCHEDULER__ */

/**
 * Class object representing IR radar module
 * Provides a high-level interface to a radar module. Supports repositioning the
 * radar's gimbal and performing a scan. A user is able to provide a function to
 * be called once a scan is complete by calling Scan(), or the scan can be
 * requested and buffers read manually using Scan(uint8_t*, uint16_t*, bool)
 */
class RadarModule
{
    friend void _RADAR_KernelCallback(void);
	public:
        static RadarModule& GetI();
        static RadarModule* GetP();

		void InitHW();
		void AddHook(void((*funPoint)(uint8_t*, uint16_t*)));

		uint32_t    Scan(uint8_t *data, uint16_t *length, bool fine);
		uint32_t    Scan(bool fine, bool hook = false);
		bool        ScanReady();
		void        ReadBuffer(uint8_t *buffer, uint16_t *bufferLen);

		void SetHorAngle(float angle);
        void SetVerAngle(float angle);

        /**
         * Hook to user routine called when the scan is complete
         * @param uint8_t* Buffer holding distance from performed scan(in cm)
         * @param uint16_t* Length of buffer (either 160(coarse) or 1280(fine))
         */
        void    ((*custHook)(uint8_t*, uint16_t*));

	protected:
        RadarModule();
        ~RadarModule();
        RadarModule(RadarModule &arg) {}          //  No definition - forbid this
        void operator=(RadarModule const &arg) {} //  No definition - forbid this

        //  Flag to signal scan being completed
		bool    _scanComplete;
		//  Dynamically allocated buffer for sensor data (allocated before scan)
		uint8_t *_scanData;
		//  Flag for user to request fine scan
		bool    _fineScan;
		//  Interface with task scheduler - provides memory space and function
		//  to call in order for task scheduler to request service from this module
#if defined(__USE_TASK_SCHEDULER__)
		_kernelEntry _radKer;
#endif
};


#endif /* RADARGP2_H_ */

