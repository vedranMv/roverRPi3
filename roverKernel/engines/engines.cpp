/**
 * engines.c
 *
 *  Created on: 29. 5. 2016.
 *      Author: Vedran
 */
#include "engines.h"

#if defined(__HAL_USE_ENGINES__)       //  Compile only if module is enabled

#include "HAL/hal.h"
#include "libs/myLib.h"

//  Enable debug information printed on serial port
//#define __DEBUG_SESSION__

//  Integration with event log, if it's present
#ifdef __HAL_USE_EVENTLOG__
    #include "init/eventLog.h"
    //  Simplify emitting events
    #define EMIT_EV(X, Y)  EventLog::EmitEvent(ENGINES_UID, X, Y)
#endif  /* __HAL_USE_EVENTLOG__ */

#ifdef __DEBUG_SESSION__
#include "serialPort/uartHW.h"
#endif

/**     Motor selectors (based on the side, viewed from the back of the vehicle) */
#define ED_LEFT     0
#define ED_RIGHT    1
#define ED_BOTH     2


#if defined(__USE_TASK_SCHEDULER__)
/**
 * Callback routine to invoke service offered by this module from task scheduler
 * @note It is assumed that once this function is called task scheduler has
 * already copied required variables into the memory space provided for it.
 */
void _ENG_KernelCallback(void)
{
    EngineData &__ed = EngineData::GetI();
    //  Check for null-pointer
    if (__ed._edKer.args == 0)
        return;
    /*
     *  Data in args[] contains bytes that constitute arguments for function
     *  calls. The exact representation(i.e. whether bytes represent ints, floats)
     *  of data is known only to individual blocks of switch() function. There
     *  is no predefined data separator between arguments inside args[].
     */
    switch (__ed._edKer.serviceID)
    {
    /*
     * Move vehicle in a single direction given by arguments
     * args[] = direction(uint8_t)|length-or-angle(4B float)|blocking(1B)
     * retVal one of myLib.h STATUS_* error codes
     */
    case ENG_T_MOVE_ENG:
        {
            uint8_t dir, temp;
            float arg;
            bool blocking;

            memcpy((void*)&dir,
                   (void*)__ed._edKer.args,
                   1);
            memcpy((void*)&arg,
                   (void*)(__ed._edKer.args + 1),
                   sizeof(float));
            memcpy((void*)&temp,
                   (void*)(__ed._edKer.args + 1 + sizeof(float)),
                   1);
            blocking = !(!temp);

            //((EngineData*)__ed)->StartEngines(dir, arg, blocking);
            __ed._edKer.retVal = __ed.StartEngines(dir, arg, blocking);
        }
        break;
    /*
     * Move vehicle following an arch
     * args[] = distance(4B float)|angle(4B float)|small-radius(4B float)
     * retVal one of myLib.h STATUS_* error codes
     */
    case ENG_T_MOVE_ARC:
        {
            float dist, angl, smallRad;

            memcpy((void*)&dist,
                   (void*)__ed._edKer.args,
                   sizeof(float));
            memcpy((void*)&angl,
                   (void*)(__ed._edKer.args + sizeof(float)),
                   sizeof(float));
            memcpy((void*)&smallRad,
                   (void*)(__ed._edKer.args + 2 * sizeof(float)),
                   sizeof(float));
            //((EngineData*)
            __ed._edKer.retVal = __ed.StartEnginesArc(dist, angl, smallRad);
        }
        break;
    /*
     * Move each wheel at given percentage of full speed
     * args[] = direction(uint8_t)|leftPercent(4B float)|rightPercent(4B float)
     * retVal one of myLib.h STATUS_* error codes
     */
    case ENG_T_MOVE_PERC:
        {
            uint8_t dir;
            float percLeft, percRight;

            memcpy((void*)&dir,
                   (void*)__ed._edKer.args,
                   1);
            memcpy((void*)&percLeft,
                   (void*)(__ed._edKer.args+1),
                   sizeof(float));
            memcpy((void*)&percRight,
                   (void*)(__ed._edKer.args + sizeof(float)+1),
                   sizeof(float));
            //((EngineData*)__ed)->RunAtPercPWM(dir, percLeft, percRight);
            __ed._edKer.retVal = __ed.RunAtPercPWM(dir, percLeft, percRight);
        }
        break;
    /*
     * Move each wheel at given percentage of full speed
     * args[] = direction(uint8_t)|leftPercent(4B float)|rightPercent(4B float)
     * retVal one of myLib.h STATUS_* error codes
     */
    case ENG_T_REBOOT:
        {
            //  Reboot only if 0x17 was sent as argument
            if (__ed._edKer.args[0] != 0x17)
                return;

            __ed._edKer.retVal = __ed.InitHW();
        }
        break;
    default:
        break;
    }

#ifdef __HAL_USE_EVENTLOG__
    if (__ed._edKer.retVal == STATUS_OK)
        EMIT_EV(__ed._edKer.serviceID, EVENT_OK);
    else
        EMIT_EV(__ed._edKer.serviceID, EVENT_ERROR);
#endif  /* __HAL_USE_EVENTLOG__ */
}

#endif

///-----------------------------------------------------------------------------
///         Functions for returning static instance                     [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Return reference to a singleton
 * @return reference to an internal static instance
 */
EngineData& EngineData::GetI()
{
    static EngineData inst;
    return inst;
}

/**
 * Return pointer to a singleton
 * @return pointer to an internal static instance
 */
EngineData* EngineData::GetP()
{
    return &(EngineData::GetI());
}

///-----------------------------------------------------------------------------
///         Other member functions                                      [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Set vehicle parameters needed to calculate motor parameters
 * @param wheelD diameter of the motorized wheels
 * @param wheelS distance(spacing) between motorized wheels
 * @param vehSiz length of the vehicle (perpendicular to wheelS)
 * @param encRes encoder resolution (ppr - points per rotation)
 */
void EngineData::SetVehSpec(
	float wheelD,
	float wheelS,
	float vehSiz,
	float encRes
){
	_wheelDia = wheelD;
	_wheelSpacing = wheelS;
	_vehicleSize = vehSiz;
	_encRes = encRes;

	wheelCounter[0] = 0;
	wheelCounter[1] = 0;
}

/**
 * Invoke initialization of hardware used by engines, makes direct call to HAL
 * @return
 */
int8_t EngineData::InitHW()
{
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_STARTUP);
#endif  /* __HAL_USE_EVENTLOG__ */
    HAL_ENG_Init(ENG_SPEED_STOP, ENG_SPEED_FULL);

    //  Listen for encoder input
    HAL_ENG_IntEnable(ED_LEFT, true);
    HAL_ENG_IntEnable(ED_RIGHT, true);

#if defined(__USE_TASK_SCHEDULER__)
    //  Register module services with task scheduler
    _edKer.callBackFunc = _ENG_KernelCallback;
    TS_RegCallback(&_edKer, ENGINES_UID);
#endif

#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_INITIALIZED);
#endif  /* __HAL_USE_EVENTLOG__ */
	return STATUS_OK;
}

///-----------------------------------------------------------------------------
///         Functions to start the motors                               [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Move vehicle in desired direction
 * @param direction - selects the direction of movement
 * @param arg - distance in centimeters(forward/backward) or angle in �(left/right)
 * 	TODO: Configure startup_ccs.c to support ISR for counters
 */
int8_t EngineData::StartEngines(uint8_t dir, float arg, bool blocking)
{
	float wheelDistance ; //centimeters

	if (!_DirValid(dir))
	    return STATUS_ARG_ERR;

	HAL_ENG_Enable(ED_BOTH, true);

 	/*
 	 * Configure PWM generators, H-bridges and set conditions to be evaluated
 	 * during movement
 	 */
	//steps_to_do = angle * PI * 2 * wheel_distance / ( 2 * 180 * circumfirance_of_wheel / 6_calibarting_points)
 	if (dir == ENG_DIR_L)
 		wheelDistance = ((float)arg * _wheelSpacing * _encRes) / (360.0  * _wheelDia);
 	//steps_to_do = angle * PI * 2 * wheel_distance / ( 2 * 180 * circumfirance_of_wheel / 6_calibarting_points)
 	else if (dir == ENG_DIR_R)
 		wheelDistance = ((float)arg * _wheelSpacing * _encRes) / (360.0  * _wheelDia);
 	//steps_to_do = distance * (circumfirance_of_wheel / 6_calibarting_points)
 	else wheelDistance = ((float)arg * _encRes)/(PI_CONST * _wheelDia);

 	if ((dir & 0x03) == 0x01 )
 	    wheelSetPoint[ED_LEFT] -= lroundf(wheelDistance);
 	else if ((dir & 0x03) == 0x02 )
 	   wheelSetPoint[ED_LEFT] += lroundf(wheelDistance);

    if ((dir & 0x0C) == 0x04 )
        wheelSetPoint[ED_RIGHT] -= lroundf(wheelDistance);
    else if ((dir & 0x0C) == 0x08 )
       wheelSetPoint[ED_RIGHT] += lroundf(wheelDistance);


#if defined(__DEBUG_SESSION__)
    DEBUG_WRITE("Going %d LEFT: %d   RIGHT: %d  \n", dir, wheelSetPoint[ED_LEFT], wheelSetPoint[ED_RIGHT]);
#endif
	HAL_ENG_SetHBridge(ED_BOTH, dir);       //  Configure H-bridge
	HAL_ENG_SetPWM(ED_LEFT, ENG_SPEED_FULL);	//  Set left engine speed
	HAL_ENG_SetPWM(ED_RIGHT, ENG_SPEED_FULL);	//  Set right engine speed


    HAL_DelayUS(1000000);

	while ( blocking && IsDriving() )
	    HAL_DelayUS(700000);
	if (blocking) HAL_ENG_Enable(ED_BOTH, false);

#if defined(__DEBUG_SESSION__)
	DEBUG_WRITE("Drove LEFT: %d   RIGHT: %d  \n", wheelCounter[ED_LEFT], wheelCounter[ED_RIGHT]);
#endif


	return STATUS_OK;	//  Successful execution
}

int8_t EngineData::RunAtPercPWM(uint8_t dir, float percLeft, float percRight)
{
	if (!_DirValid(dir))
        return STATUS_ARG_ERR;
	HAL_ENG_Enable(ED_BOTH, true);

	//	Set to non-zero number to indicate that motors are running
	wheelSetPoint[ED_LEFT] = 1;
	wheelSetPoint[ED_RIGHT] = 1;


	if (percLeft >= 100)
	    HAL_ENG_SetPWM(ED_LEFT, ENG_SPEED_FULL);
	if (percLeft <= 0 )
	    HAL_ENG_SetPWM(ED_LEFT, ENG_SPEED_STOP);
	else HAL_ENG_SetPWM(ED_LEFT, (percLeft*0.75/100 + 0.25) * ENG_SPEED_FULL);

	if (percRight >= 100)
	    HAL_ENG_SetPWM(ED_RIGHT, ENG_SPEED_FULL);
	if (percRight <= 0 )
	    HAL_ENG_SetPWM(ED_RIGHT, ENG_SPEED_STOP);
	else HAL_ENG_SetPWM(ED_RIGHT, (percRight*0.75/100 + 0.25) * ENG_SPEED_FULL);

	if (HAL_ENG_GetHBridge(ED_BOTH) != dir)
	    HAL_ENG_SetHBridge(ED_BOTH, dir);


	return STATUS_OK;	//  Successful execution
}

/**
 *  Move vehicle over a circular path
 * 	@param distance - distance ALONG THE CIRCUMFERENCE of arc that's necessary to travel
 * 	@param angle - angle in �(left/right) that's needed to travel along the arc
 * 	@param smallRadius - radius that's going to be traveled by the inner wheel (smaller comparing to outter wheel)
 * 	Function can be called by only two of the arguments(leaving third 0) as arc parameters can be calculated based on:
 * 		-angle and distance
 * 		-angle and small radius
 * 	Path is calculated based on the value of smallRadius. If it's 0 it will be calculated from distance and vehicle size
 * 	TODO: Configure startup_ccs.c to support ISR for counters
 */
int8_t EngineData::StartEnginesArc(float distance, float angle, float smallRadius)
{
	float speedFactor = 1;

 	//  One of those parameters is needed to be non-zero to calculate valid path
 	if ( (distance == 0.0f) && (smallRadius == 0.0f))
 	        return STATUS_ARG_ERR;
 	HAL_ENG_Enable(ED_BOTH, true);

 	wheelSetPoint[0] = wheelCounter[0];
 	wheelSetPoint[1] = wheelCounter[1];

 	//steps_to_do = angle * PI * 2 * wheel_distance / ( 2 * 180 * circumfirance_of_wheel / 6_calibarting_points)
	if (angle > 90)
 	{
 		//distance = distance - wheelSafety[ED_LEFT][0] * _wheelDia * PI_CONST/_encRes;
 		angle-= 90;
 		if (smallRadius == 0.0) smallRadius = (distance/sin(angle*PI_CONST/180) - _wheelSpacing/2);
 		wheelSetPoint[ED_LEFT] += lroundf((smallRadius * (angle*PI_CONST)/180) * _encRes/(PI_CONST*_wheelDia));
 		wheelSetPoint[ED_RIGHT] += lroundf(((smallRadius + _wheelSpacing)*(angle*PI_CONST)/180) * _encRes/(PI_CONST*_wheelDia));

 		//  Speed difference between the engines
 		speedFactor = (float)wheelSetPoint[ED_LEFT]/((float)wheelSetPoint[ED_RIGHT]);

 		//  Set right & left engine speed
 		HAL_ENG_SetPWM(ED_RIGHT, ENG_SPEED_FULL);
 		HAL_ENG_SetPWM(ED_LEFT, ENG_SPEED_FULL * speedFactor * 0.9);
 	}
 	else
 	{
 		//distance = distance - wheelSafety[ED_RIGHT][0] * _wheelDia * PI_CONST/_encRes;
 		angle = 90 - angle;
 		if (smallRadius == 0.0) smallRadius = (distance/sin(angle*PI_CONST/180) - _wheelSpacing/2);
 		wheelSetPoint[ED_RIGHT] += lroundf((smallRadius * (angle*PI_CONST)/180)*_encRes/(PI_CONST*_wheelDia));
 		wheelSetPoint[ED_LEFT] += lroundf(((smallRadius + _wheelSpacing)*(angle*PI_CONST)/180)*_encRes/(PI_CONST*_wheelDia));

 		//  Speed difference between the engines
 		speedFactor = (float)wheelSetPoint[ED_RIGHT]/((float)wheelSetPoint[ED_LEFT]);

 		//  Set right & left engine speed
 		HAL_ENG_SetPWM(ED_LEFT, ENG_SPEED_FULL);
 		HAL_ENG_SetPWM(ED_RIGHT, ENG_SPEED_FULL * speedFactor *0.9);
 	}

	HAL_ENG_SetHBridge(ED_BOTH, ENG_DIR_FW);

	while ( IsDriving() )
	    HAL_DelayUS(700000);

	HAL_ENG_Enable(ED_BOTH, true);


	return STATUS_OK;	//  Successful execution
}

/**
 * Return true if the vehicle is driving at the moment
 * 		Check is performed by reading wheel counters for each wheel
 */
bool EngineData::IsDriving() volatile
{
    static int32_t ref[0];
    bool retVal = false;

    if ((wheelCounter[ED_LEFT] != ref[ED_LEFT]) || (wheelCounter[ED_RIGHT] != ref[ED_RIGHT]))
        retVal = true;

    ref[ED_LEFT] = wheelCounter[ED_LEFT];
    ref[ED_RIGHT] = wheelCounter[ED_RIGHT];

    return retVal;
}

/**
 * Check if the passed arguments are valid
 */
bool EngineData::_DirValid(uint8_t dir)
{
	if ((dir == ENG_DIR_BW) || (dir == ENG_DIR_FW) ||
	    (dir == ENG_DIR_L) || (dir == ENG_DIR_R))
		return true;
	return false;
}

///-----------------------------------------------------------------------------
///         Class constructor and destructor                         [PROTECTED]
///-----------------------------------------------------------------------------

EngineData::EngineData()
{
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_UNINITIALIZED);
#endif  /* __HAL_USE_EVENTLOG__ */
}
EngineData::~EngineData() {}


///-----------------------------------------------------------------------------
///         Declaration of ISR functions for optical encoders          [PRIVATE]
///* Function called every time an encoder gives a pulse for a rotating wheel
///* Decreases internal counter for corresponding wheel and at counter=1 stop
///* rotation and let the vehicle stop by its own weight
///-----------------------------------------------------------------------------

/**
 * ISR for left engine
 */
void PP0ISR(void)
{
    EngineData *__ed = EngineData::GetP();
    HAL_ENG_IntClear(ED_LEFT);

    //if (__ed->wheelCounter[ED_LEFT] > 0)
    if (HAL_ENG_GetHBridge(ED_LEFT) == 0x01)    //0b00000001
            __ed->wheelCounter[ED_LEFT]--;
    else if (HAL_ENG_GetHBridge(ED_LEFT) == 0x02)   //0b00000010
        __ed->wheelCounter[ED_LEFT]++;

    /*if (__ed->wheelCounter[ED_LEFT] == 1)
    {
        HAL_ENG_SetHBridge(ED_LEFT, ~HAL_ENG_GetHBridge(ED_LEFT));
    }
    else*/ if (labs(__ed->wheelCounter[ED_LEFT] - __ed->wheelSetPoint[ED_LEFT]) < 1)
    {
        HAL_ENG_SetPWM(ED_LEFT, ENG_SPEED_STOP);
        HAL_ENG_Enable(ED_LEFT, false);
    }
}

/**
 * ISR for right engine
 */
void PP1ISR(void)
{
    EngineData *__ed = EngineData::GetP();
    HAL_ENG_IntClear(ED_RIGHT);

    //if (__ed->wheelCounter[ED_RIGHT] > 0)
    //        __ed->wheelCounter[ED_RIGHT]--;
    if (HAL_ENG_GetHBridge(ED_RIGHT) == 0x04)    //0b00000100
            __ed->wheelCounter[ED_RIGHT]--;
    else if (HAL_ENG_GetHBridge(ED_RIGHT) == 0x08)   //0b00001000
        __ed->wheelCounter[ED_RIGHT]++;


    /*if (__ed->wheelCounter[ED_RIGHT] == 3)
    {
        HAL_ENG_SetHBridge(ED_RIGHT, ~HAL_ENG_GetHBridge(ED_RIGHT));
    }
    else*/ if (labs(__ed->wheelCounter[ED_RIGHT] - __ed->wheelSetPoint[ED_RIGHT]) < 1)
    {
        HAL_ENG_SetPWM(ED_RIGHT, ENG_SPEED_STOP);
        HAL_ENG_Enable(ED_RIGHT, false);
    }
}

#endif  /* __HAL_USE_ENGINES__ */
