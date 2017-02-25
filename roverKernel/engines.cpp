/**
 * engines.c
 *
 *  Created on: 29. 5. 2016.
 *      Author: Vedran
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "engines.h"
#include "tm4c1294_hal.h"
#include "myLib.h"

#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "utils/uartstdio.h"

//  Global pointer to FIRST created instance of EngineData
volatile EngineData* __ed;

#if defined(__USE_TASK_SCHEDULER__)
/**
 * Callback routine to invoke service offered by this module from task scheduler
 * @note It is assumed that once this function is called task scheduler has
 * already copied required variables into the memory space provided for it.
 */
void _ENG_KernelCallback(void)
{
    //  Check for null-pointer
    if (__ed->_edKer.argN == 0)
        return;
    /*
     *  Data in args[] contains bytes that constitute arguments for function
     *  calls. The exact representation(i.e. whether bytes represent ints, floats)
     *  of data is known only to individual blocks of switch() function. There
     *  is no predefined data separator between arguments inside args[].
     */
    switch (__ed->_edKer.serviceID)
    {
    /*
     * Move vehicle in a single direction given by arguments
     * args[] = direction(uint8_t)|length-or-angle(4B float)|blocking(1B)
     */
    case ENG_MOVE_ENG:
        {
            uint8_t dir, temp;
            float arg;
            bool blocking;

            memcpy((void*)&dir,
                   (void*)__ed->_edKer.args,
                   1);
            memcpy((void*)&arg,
                   (void*)(__ed->_edKer.args + 1),
                   sizeof(float));
            memcpy((void*)&temp,
                   (void*)(__ed->_edKer.args + 1 + sizeof(float)),
                   1);
            blocking = !(!temp);

            ((EngineData*)__ed)->StartEngines(dir, arg, blocking);
        }
        break;
    /*
     * Move vehicle following an arch
     * args[] = distance(4B float)|angle(4B float)|small-radius(4B float)
     */
    case ENG_MOVE_ARC:
        {
            float dist, angl, smallRad;

            memcpy((void*)&dist,
                   (void*)__ed->_edKer.args,
                   sizeof(float));
            memcpy((void*)&angl,
                   (void*)(__ed->_edKer.args + sizeof(float)),
                   sizeof(float));
            memcpy((void*)&smallRad,
                   (void*)(__ed->_edKer.args + 2 * sizeof(float)),
                   sizeof(float));
            ((EngineData*)__ed)->StartEnginesArc(dist, angl, smallRad);
        }
        break;
    /*
     * Move each wheel at given percentage of full speed
     * args[] = direction(uint8_t)|leftPercent(4B float)|rightPercent(4B float)
     */
    case ENG_MOVE_PERC:
        {
            uint8_t dir;
            float percLeft, percRight;

            memcpy((void*)&dir,
                   (void*)__ed->_edKer.args,
                   1);
            memcpy((void*)&percLeft,
                   (void*)(__ed->_edKer.args+1),
                   sizeof(float));
            memcpy((void*)&percRight,
                   (void*)(__ed->_edKer.args + sizeof(float)+1),
                   sizeof(float));
            ((EngineData*)__ed)->RunAtPercPWM(dir, percLeft, percRight);
        }
        break;
    default:
        break;
    }
}


#endif


EngineData::EngineData()
{
	__ed = this;
}
EngineData::EngineData(
	float wheelD,
	float wheelS,
	float vehSiz,
	float encRes)
	: _wheelDia(wheelD), _wheelSpacing(wheelS), _vehicleSize(vehSiz),
	  _encRes(encRes)
{
	__ed = this;
}

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
 * ISR for left engine
 * 		Function called every time an encoder gives a pulse for a rotating
 * 		wheel. Decreases internal counter for corresponding wheel and slows down
 * 		rotation by 50% before stopping to minimize position overshoot on halt
 * 	TODO: Implement a stack of distances for each wheel so the vehicle can
 * 		perform more complex maneuvers
 */
void PP0ISR(void)
{
    HAL_ENG_IntClear(ED_LEFT);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, ~GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2));

    if (__ed->wheelCounter[ED_LEFT] > 0)
            __ed->wheelCounter[ED_LEFT]--;

    /*if (__ed->wheelCounter[ED_LEFT] == 1)
    {
        HAL_ENG_SetHBridge(ED_LEFT, ~HAL_ENG_GetHBridge(ED_LEFT));
    }
    else*/ if (__ed->wheelCounter[ED_LEFT] == 0)
    {
        HAL_ENG_SetPWM(ED_LEFT, ENGINE_STOP);
        HAL_ENG_Enable(ED_LEFT, false);
    }
}

/**
 * ISR for right engine
 * 		Function called every time an encoder gives a pulse for a rotating
 * 		wheel. Decreases internal counter for corresponding wheel and slows down
 * 		rotation by 50% before stopping to minimize position overshoot on halt
 * 	TODO: Implement a stack of distances for each wheel so the vehicle can
 * 		perform more complex maneuvers
 */
void PP1ISR(void)
{
    HAL_ENG_IntClear(ED_RIGHT);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, ~GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_3));

    if (__ed->wheelCounter[ED_RIGHT] > 0)
            __ed->wheelCounter[ED_RIGHT]--;

    /*if (__ed->wheelCounter[ED_RIGHT] == 3)
    {
        HAL_ENG_SetHBridge(ED_RIGHT, ~HAL_ENG_GetHBridge(ED_RIGHT));
    }
    else*/ if (__ed->wheelCounter[ED_RIGHT] == 1)
    {
        HAL_ENG_SetPWM(ED_RIGHT, ENGINE_STOP);
        HAL_ENG_Enable(ED_RIGHT, false);
    }
}


int8_t EngineData::InitHW()
{
    HAL_ENG_Init(ENGINE_STOP, ENGINE_FULL);

    //  Listen for encoder input
    HAL_ENG_IntEnable(ED_LEFT, true);
    HAL_ENG_IntEnable(ED_RIGHT, true);

#if defined(__USE_TASK_SCHEDULER__)
    //  Register module services with task scheduler
    _edKer.callBackFunc = _ENG_KernelCallback;
    TS_RegCallback(&_edKer, ENGINES_UID);
#endif
	return STATUS_OK;
}

/**
 * Move vehicle in desired direction
 * 		@param direction - selects the direction of movement
 * 		@param arg - distance in centimeters(forward/backward) or angle in �(left/right)
 * 	TODO: Configure startup_ccs.c to support ISR for counters
 */
int8_t EngineData::StartEngines(uint8_t dir, float arg, bool blocking)
{
	float wheelDistance ; //centimeters

	if (!_DirValid(dir)) return STATUS_ARG_ERR;

	HAL_ENG_Enable(ED_BOTH, true);
 	/**
 	 * Configure PWM generators, H-bridges and set conditions to be evaluated
 	 * during movement
 	 */

	//steps_to_do = angle * PI * 2 * wheel_distance / ( 2 * 180 * circumfirance_of_wheel / 6_calibarting_points)
 	if (dir == DIRECTION_LEFT)
 		wheelDistance = ((float)arg * _wheelSpacing * _encRes) / (360.0  * _wheelDia);
 	//steps_to_do = angle * PI * 2 * wheel_distance / ( 2 * 180 * circumfirance_of_wheel / 6_calibarting_points)
 	else if (dir == DIRECTION_RIGHT)
 		wheelDistance = ((float)arg * _wheelSpacing * _encRes) / (360.0  * _wheelDia);
 	//steps_to_do = distance * (circumfirance_of_wheel / 6_calibarting_points)
 	else wheelDistance = ((float)arg * _encRes)/(PI_CONST * _wheelDia);

 	wheelCounter[ED_LEFT] = lroundf(wheelDistance);

 	wheelCounter[ED_RIGHT] = wheelCounter[ED_LEFT];	//set counter for right engine


#if defined(__DEBUG_SESSION__)
	UARTprintf("Going %d LEFT: %d   RIGHT: %d  \n", dir, wheelCounter[ED_LEFT], wheelCounter[ED_RIGHT]);
#endif
	HAL_ENG_SetHBridge(ED_BOTH, dir);       //configure H-bridge
	HAL_ENG_SetPWM(ED_LEFT, ENGINE_FULL);	//set left engine speed
	HAL_ENG_SetPWM(ED_RIGHT, ENGINE_FULL);	//set right engine speed

	//HAL_ENG_IntEnable(ED_RIGHT, true);

	while ( blocking && IsDriving() );
	if (blocking) HAL_ENG_Enable(ED_BOTH, false);

	return STATUS_OK;	//Successful execution
}

int8_t EngineData::RunAtPercPWM(uint8_t dir, float percLeft, float percRight)
{
	if (!_DirValid(dir)) return STATUS_ARG_ERR;
	HAL_ENG_Enable(ED_BOTH, true);

	//	Set to non-zero number to indicate that motors are running
	wheelCounter[ED_LEFT] = 1;
	wheelCounter[ED_RIGHT] = 1;


	if (percLeft >= 100)
	    HAL_ENG_SetPWM(ED_LEFT, ENGINE_FULL);
	if (percLeft <= 0 )
	    HAL_ENG_SetPWM(ED_LEFT, ENGINE_STOP);
	else HAL_ENG_SetPWM(ED_LEFT, (percLeft*0.75/100 + 0.25) * ENGINE_FULL);

	if (percRight >= 100)
	    HAL_ENG_SetPWM(ED_RIGHT, ENGINE_FULL);
	if (percRight <= 0 )
	    HAL_ENG_SetPWM(ED_RIGHT, ENGINE_STOP);
	else HAL_ENG_SetPWM(ED_RIGHT, (percRight*0.75/100 + 0.25) * ENGINE_FULL);

	if (HAL_ENG_GetHBridge(ED_BOTH) != dir)
	    HAL_ENG_SetHBridge(ED_BOTH, dir);

	return STATUS_OK;	//Successful execution
}

/**
 * Move vehicle over a circular path
 * 		@param distance - distance ALONG THE CIRCUMFERENCE of arc that's necessary to travel
 * 		@param angle - angle in �(left/right) that's needed to travel along the arc
 * 		@param smallRadius - radius that's going to be traveled by the inner wheel (smaller comparing to outter wheel)
 * 	Function can be called by only two of the arguments(leaving third 0) as arc parameters can be calculated based on:
 * 		-angle and distance
 * 		-angle and small radius
 * 	Path is calculated based on the value of smallRadius. If it's 0 it will be calculated from distance and vehicle size
 * 	TODO: Configure startup_ccs.c to support ISR for counters
 */
int8_t EngineData::StartEnginesArc(float distance, float angle, float smallRadius)
{
	float speedFactor = 1;

 	//One of those parameters is needed to be non-zero to calculate valid path
 	if ( (distance == 0.0f) && (smallRadius == 0.0f)) return STATUS_ARG_ERR;
 	HAL_ENG_Enable(ED_BOTH, true);

	if (angle > 90)//steps_to_do = angle * PI * 2 * wheel_distance / ( 2 * 180 * circumfirance_of_wheel / 6_calibarting_points)
 	{
 		//distance = distance - wheelSafety[ED_LEFT][0] * _wheelDia * PI_CONST/_encRes;
 		angle-= 90;
 		if (smallRadius == 0.0) smallRadius = (distance/sin(angle*PI_CONST/180) - _wheelSpacing/2);
 		wheelCounter[ED_LEFT] = lroundf((smallRadius * (angle*PI_CONST)/180) * _encRes/(PI_CONST*_wheelDia));
 		wheelCounter[ED_RIGHT] = lroundf(((smallRadius + _wheelSpacing)*(angle*PI_CONST)/180) * _encRes/(PI_CONST*_wheelDia));

 		speedFactor = (float)wheelCounter[ED_LEFT]/((float)wheelCounter[ED_RIGHT]);

 		HAL_ENG_SetPWM(ED_RIGHT, ENGINE_FULL);	//set right engine speed
 		HAL_ENG_SetPWM(ED_LEFT, ENGINE_FULL * speedFactor * 0.9);	//set left engine speed
 	}
 	else
 	{
 		//distance = distance - wheelSafety[ED_RIGHT][0] * _wheelDia * PI_CONST/_encRes;
 		angle = 90 - angle;
 		if (smallRadius == 0.0) smallRadius = (distance/sin(angle*PI_CONST/180) - _wheelSpacing/2);
 		wheelCounter[ED_RIGHT] = lroundf((smallRadius * (angle*PI_CONST)/180)*_encRes/(PI_CONST*_wheelDia));
 		wheelCounter[ED_LEFT] = lroundf(((smallRadius + _wheelSpacing)*(angle*PI_CONST)/180)*_encRes/(PI_CONST*_wheelDia));


 		speedFactor = (float)wheelCounter[ED_RIGHT]/((float)wheelCounter[ED_LEFT]);	//Speed difference between the engines

 		HAL_ENG_SetPWM(ED_LEFT, ENGINE_FULL);	//set right engine speed
 		HAL_ENG_SetPWM(ED_RIGHT, ENGINE_FULL * speedFactor *0.9);	//set left engine speed
 	}

	HAL_ENG_SetHBridge(ED_BOTH, DIRECTION_FORWARD);

    HAL_ENG_IntEnable(ED_LEFT, true);
    HAL_ENG_IntEnable(ED_RIGHT, true);

	while ( IsDriving() );
	HAL_ENG_Enable(ED_BOTH, true);

	return STATUS_OK;	//Successful execution
}

/**
 * Return true if the vehicle is driving at the moment
 * 		Check is performed by reading wheel counters for each wheel
 */
bool EngineData::IsDriving() volatile
{
	return (wheelCounter[ED_LEFT] > 0) || (wheelCounter[ED_RIGHT] > 0);
}

/**
 * Check if the passed arguments are valid
 */
bool EngineData::_DirValid(uint8_t dir)
{
	if ((dir == DIRECTION_BACKWARD) || (dir == DIRECTION_FORWARD)
			|| (dir == DIRECTION_LEFT) || (dir == DIRECTION_RIGHT))
		return true;
	return false;
}

