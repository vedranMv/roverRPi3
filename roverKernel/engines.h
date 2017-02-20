/**
 * engines.h
 *
 *  Created on: 29. 5. 2016.
 *      Author: Vedran
 *  @version v2.1
 *  V1.0 - 29.5.2016
 *  +Implemented C code as C++ object, adjusted it to use HAL
 *  V2.0 - 7.2.2017
 *  +Integration of library with task scheduler
 *
 *
 ****Hardware dependencies:
 * 	PF2/PF3 - PWMOut2/PWMOut3 - Control left/right engine PWM signal (PWM block 0, Generator 1, Outputs 2,3)
 * 		Gen1 generates ~60Hz PWM (31249 passed as argument)	-could be increased to get more power????
 * 		PWMOut2/PWMOut3 run between 5 and 31249
 * 	PL0/PL1/PL2/PL3 - GPIO - H-bridge configuration for engines
 * 		PL1-PL0 -> sets direction of left engine (see direction Macros)
 * 		PL3-PL2 -> sets direction of right engine (see direction Macros)
 * 	PP0/PP1 - GPIO - Optical encoders for each wheel, resolution of 90 slits per rotation
 * 		Interrupt based
 * 		PP0 -> counts left engine
 * 		PP1 -> counts right engine
 *	IMPORTANT: PWM module runs with clock divider of 32, should it be changed to
 *		e.g. 64, all number passed to PWM have to be halved
 *		Interrupts have to be registered through startup_ccs.c file
 */

#ifndef ENGINES_H_
#define ENGINES_H_

//  Enable debug information printed on serial port
//#define __DEBUG_SESSION__
//  Enable integration of this library with task scheduler
#define __USE_TASK_SCHEDULER__

#if defined(__USE_TASK_SCHEDULER__)
    #include "taskScheduler.h"
    //  Unique identifier of this module as registered in task scheduler
    #define ENGINES_UID         0
    //  Definitions of ServiceID for service offered by this module
    #define ENG_SET_SPEED       0   //  Update speed on L&R engine
    #define ENG_SET_DIST        1   //  Update distance for L&R wheel

#endif
/*
 * Movement direction definitions for H-bridge
 */
#define ENGINE_STOP 		1		//PWM argument for stopping engine
#define ENGINE_FULL 		15000	//PWM argument for engine full-speed
#define ENGINE_FULL_ARG 	18750

#define DIRECTION_FORWARD 	0x0A	//move forward H-bridge configuration  1010
#define DIRECTION_BACKWARD 	0x05	//move backward H-bridge configuration 0101
#define DIRECTION_LEFT 		0x09	//turn left H-bridge configuration 1001
#define DIRECTION_RIGHT 	0x06	//turn right H-bridge configuration 0110

#define DIR_WHEEL_FWD       0x02
#define DIR_WHEEL_BCK       0x01

#define ED_LEFT		0
#define ED_RIGHT 	1
#define ED_BOTH     2

class EngineData
{
    friend void ControlLoop(void);
    friend void _ENG_KernelCallback(void);
	public:
		EngineData();
		EngineData(float wheelD, float wheelS, float vehSiz, float encRes);
		void SetVehSpec(float wheelD, float wheelS, float vehSiz, float encRes);

		int8_t 	InitHW();
		int8_t 	StartEngines(uint8_t direction, float arg, bool blocking = true);
		int8_t 	RunAtPercPWM(uint8_t dir, float percLeft, float percRight);
		int8_t 	StartEnginesArc(float distance, float angle, float smallRadius);
		bool 	IsDriving() volatile;
		void 	SetSafetySeq(uint8_t seq, uint32_t right, uint32_t left);

		volatile int32_t wheelCounter[2];   //  In encoder ticks

	protected:
		bool _DirValid(uint8_t dir);
		uint32_t _cmpsToEncT(float &ticks);

		//  Mechanical properties of platform
		float _wheelDia; 	//in cm
		float _wheelSpacing; //in cm
		float _vehicleSize; 	//in cm
		float _encRes;	//Encoder resolution in points (# of points/rotation)

        //  Interface with task scheduler - provides memory space and function
        //  to call in order for task scheduler to request service from this module
#if defined(__USE_TASK_SCHEDULER__)
        _kernelEntry _edKer;
#endif
};

extern volatile EngineData* __ed;

extern "C"
{
	extern void PP0ISR(void);
	extern void PP1ISR(void);
}

#endif /* ENGINES_H_ */
