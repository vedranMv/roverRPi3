/**
 * engines.h
 *
 *  Created on: 29. 5. 2016.
 *      Author: Vedran Mikov
 *  @version v2.1.1
 *  V1.0 - 29.5.2016
 *  +Implemented C code as C++ object, adjusted it to use HAL
 *  V2.0 - 7.2.2017
 *  +Integration of library with task scheduler
 *  V2.1 - 25.2.2017
 *  +Completed kernel callback function, now supports calls to all functions
 *  offered by the EngineData kernel module
 *  V2.1.1 - 7.3.2016
 *  +Changed EngineData class into a singleton
 */
#include "roverKernel/hwconfig.h"

//  Compile following section only if hwconfig.h says to include this module
#if !defined(ENGINES_H_) && defined(__HAL_USE_ENGINES__)
#define ENGINES_H_

//  Enable debug information printed on serial port
//#define __DEBUG_SESSION__

//  Enable integration of this library with task scheduler but only if task
//  scheduler is being compiled into this project
#if defined(__HAL_USE_TASKSCH__)
#define __USE_TASK_SCHEDULER__
#endif  /* __HAL_USE_TASKSCH__ */

//  Check if this library is set to use task scheduler
#if defined(__USE_TASK_SCHEDULER__)
    #include "roverKernel/taskScheduler/taskScheduler.h"
    //  Unique identifier of this module as registered in task scheduler
    #define ENGINES_UID         2
    //  Definitions of ServiceID for service offered by this module
    #define ENG_MOVE_ENG        0
    #define ENG_MOVE_ARC        1
    #define ENG_MOVE_PERC       2

#endif

/**     PWM arguments for different motor speed */
#define ENGINE_STOP 		1		//PWM argument for stopping engine
#define ENGINE_FULL 		15000	//PWM argument for engine full-speed
#define ENGINE_FULL_ARG 	18750

/**     Movement direction definitions for H-bridge - Direction macros  */
#define DIRECTION_FORWARD 	0x0A	//  Move forward H-bridge configuration  1010
#define DIRECTION_BACKWARD 	0x05	//  Move backward H-bridge configuration 0101
#define DIRECTION_LEFT 		0x09	//  Turn left H-bridge configuration 1001
#define DIRECTION_RIGHT 	0x06	//  Turn right H-bridge configuration 0110

/**     Motor selectors (based on the side, viewed from the back of the vehicle) */
#define ED_LEFT		0
#define ED_RIGHT 	1
#define ED_BOTH     2

class EngineData
{
    friend void ControlLoop(void);
    friend void _ENG_KernelCallback(void);
	public:
        static EngineData& GetI();
        static EngineData* GetP();

		void SetVehSpec(float wheelD, float wheelS, float vehSiz, float encRes);

		int8_t 	InitHW();
		int8_t 	StartEngines(uint8_t direction, float arg, bool blocking = true);
		int8_t 	RunAtPercPWM(uint8_t dir, float percLeft, float percRight);
		int8_t 	StartEnginesArc(float distance, float angle, float smallRadius);
		bool 	IsDriving() volatile;
		void 	SetSafetySeq(uint8_t seq, uint32_t right, uint32_t left);

		//  Number of encoder counts each wheel has traveled
		//  (+ for forward direction, - for backward)
		volatile int32_t wheelCounter[2];   //  In encoder ticks

	protected:
        EngineData();
        ~EngineData();
        EngineData(float wheelD, float wheelS, float vehSiz, float encRes);
        EngineData(EngineData &arg) {}           //  No definition - forbid this
        void operator=(EngineData const &arg) {} //  No definition - forbid this

		bool _DirValid(uint8_t dir);
		uint32_t _cmpsToEncT(float &ticks);

		//  Mechanical properties of platform
		float _wheelDia;        //in cm
		float _wheelSpacing;    //in cm
		float _vehicleSize; 	//in cm
		float _encRes;	//  Encoder resolution in points (# of points/rotation)

        //  Interface with task scheduler - provides memory space and function
        //  to call in order for task scheduler to request service from this module
#if defined(__USE_TASK_SCHEDULER__)
        _kernelEntry _edKer;
#endif
};


extern "C"
{
	extern void PP0ISR(void);
	extern void PP1ISR(void);
}

#endif /* ENGINES_H_ */
