/**
 * engines.h
 *
 *  Created on: 29. 5. 2016.
 *      Author: Vedran Mikov
 *  @version v2.1.3
 *  V1.0 - 29.5.2016
 *  +Implemented C code as C++ object, adjusted it to use HAL
 *  V2.0 - 7.2.2017
 *  +Integration of library with task scheduler
 *  V2.1 - 25.2.2017
 *  +Completed kernel callback function, now supports calls to all functions
 *  offered by the EngineData kernel module
 *  V2.1.1 - 7.3.2017
 *  +Changed EngineData class into a singleton
 *  V2.1.2 - 21.3.2017
 *  +Total distance traveled by each wheel separated(counter) from set point
 *  V2.1.3 - 30.6.2017
 *  +Change include paths for better portability, new way of printing to debug
 *  +Integration with event logger
 */
#include "hwconfig.h"

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
    #include "taskScheduler/taskScheduler.h"
    //  Unique identifier of this module as registered in task scheduler
    #define ENGINES_UID         2
    //  Definitions of ServiceID for service offered by this module
    #define ENG_T_MOVE_ENG        0
    #define ENG_T_MOVE_ARC        1
    #define ENG_T_MOVE_PERC       2

#endif

/**     PWM arguments for different motor speed */
#define ENG_SPEED_STOP 		1		//PWM argument for stopping engine
#define ENG_SPEED_FULL 		15000	//PWM argument for engine full-speed
#define ENGINE_FULL_ARG 	18750

/**     Movement direction definitions for H-bridge - Direction macros  */
#define ENG_DIR_FW 	0x0A	//  Move forward H-bridge configuration  1010
#define ENG_DIR_BW 	0x05	//  Move backward H-bridge configuration 0101
#define ENG_DIR_L 	0x09	//  Turn left H-bridge configuration     1001
#define ENG_DIR_R 	0x06	//  Turn right H-bridge configuration    0110

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
		//  Desired position for each wheel
		volatile int32_t wheelSetPoint[2];  //  In encoder ticks

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
