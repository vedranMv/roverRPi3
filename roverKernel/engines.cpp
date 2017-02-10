/*
 * engines.c
 *
 *  Created on: 29. 5. 2016.
 *      Author: Vedran
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "engines.h"
#include "tm4c1294_hal.h"
#include "myLib.h"

#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "utils/uartstdio.h"


volatile EngineData* __pED;
int32_t engineOffset=0;	//Error in measurements between optical encoders - 0


EngineData::EngineData()
{
	__pED = this;
}
EngineData::EngineData(
	float wheelD,
	float wheelS,
	float vehSiz,
	float encRes)
	: _wheelDia(wheelD), _wheelSpacing(wheelS), _vehicleSize(vehSiz),
	  _encRes(encRes)
{
	__pED = this;
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
	safetyCounter[0] = 0;
	safetyCounter[1] = 0;
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
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, ~GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0));
    //UARTprintf("RIGHT: %d\n", __pED->wheelCounter[ED_RIGHT]);

	if (__pED->wheelCounter[ED_LEFT] > 0)
	{
		__pED->wheelCounter[ED_LEFT]--;
		/*if (__pED->wheelCounter[ED_LEFT] <=
		 		__pED->wheelSafety[ED_LEFT][__pED->safetyCounter[ED_LEFT]])
		{
			if (__pED->safetyCounter[ED_LEFT] > 1) __pED->safetyCounter[ED_LEFT]--;
			PWMPulseWidthSet(ED_PWM_BASE, ED_PWM_LEFT, ENGINE_FULL * 0.55);
		}*/
	}

	if (__pED->wheelCounter[ED_LEFT] == 0)
	{
	    HAL_ENG_IntEnable(ED_LEFT, false);
	    HAL_ENG_SetHBridge(ED_LEFT, ~HAL_ENG_GetHBridge(ED_LEFT));
	    HAL_ENG_SetPWM(ED_LEFT, ENGINE_STOP);
	}
	/*else if ( (__pED->wheelCounter[ED_LEFT] < 10)
				&& (__pED->wheelCounter[ED_LEFT] > 0) )
	{
	    HAL_ENG_SetPWM(ED_LEFT, 0.50 * ENGINE_FULL);
		//PWMPulseWidthSet(ED_PWM_BASE, ED_PWM_LEFT, 0.50 * ENGINE_FULL);
	}*/
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
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, ~GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_1));
    //UARTprintf("LEFT: %d\n", __pED->wheelCounter[ED_RIGHT]);

	if (__pED->wheelCounter[ED_RIGHT] > 0)
	{
		__pED->wheelCounter[ED_RIGHT]--;
		/*if (__pED->wheelCounter[ED_RIGHT] <=
		 	 	 __pED->wheelSafety[ED_RIGHT][__pED->safetyCounter[ED_RIGHT]])
		{
			if (__pED->safetyCounter[ED_RIGHT] > 0) __pED->safetyCounter[ED_RIGHT]--;
			PWMPulseWidthSet(ED_PWM_BASE, ED_PWM_RIGHT, ENGINE_FULL * 0.60);
		}*/
	}

	if (__pED->wheelCounter[ED_RIGHT] == 0)
	{
        HAL_ENG_IntEnable(ED_RIGHT, false);
        HAL_ENG_SetHBridge(ED_RIGHT, ~HAL_ENG_GetHBridge(ED_RIGHT));
        HAL_ENG_SetPWM(ED_RIGHT, ENGINE_STOP);
	}
	/*else if ( (__pED->wheelCounter[ED_RIGHT] < 10)
				&& (__pED->wheelCounter[ED_RIGHT] > 0) )
	{
	    HAL_ENG_SetPWM(ED_RIGHT, 0.52 * ENGINE_FULL);
		//PWMPulseWidthSet(ED_PWM_BASE, ED_PWM_RIGHT, 0.52 * ENGINE_FULL);
	}*/
}

int8_t EngineData::InitHW()
{
    HAL_ENG_Init(ENGINE_STOP, ENGINE_FULL);

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
	//char message[300];

	if (!_DirValid(dir)) return STATUS_ARG_ERR;

	HAL_ENG_Enable(true);
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

 	wheelCounter[ED_LEFT] = (uint32_t)roundf(wheelDistance);

 	wheelCounter[ED_RIGHT] = wheelCounter[ED_LEFT];	//set counter for right engine
 	wheelCounter[ED_LEFT] -= engineOffset;

	//if any wheel engine is turned off, don't count for it
	if ( (dir & 0x03) == 0 ) wheelCounter[ED_LEFT] = 0;
	if ( ((dir >> 2) & 0x03) == 0 ) wheelCounter[ED_RIGHT] = 0;

	UARTprintf("Going %d LEFT: %d   RIGHT: %d  \n", dir, wheelCounter[ED_LEFT], wheelCounter[ED_RIGHT]);
	HAL_ENG_SetPWM(ED_LEFT, ENGINE_FULL);	//set left engine speed
	HAL_ENG_SetPWM(ED_RIGHT, ENGINE_FULL);	//set right engine speed
	HAL_ENG_SetHBridge(ED_BOTH, dir);       //configure H-bridge


	HAL_ENG_IntEnable(ED_LEFT, true);
	HAL_ENG_IntEnable(ED_RIGHT, true);

	while ( blocking && IsDriving() );
	if (blocking) HAL_ENG_Enable(false);

	return STATUS_OK;	//Successful execution
}

int8_t EngineData::RunAtPercPWM(uint8_t dir, float percLeft, float percRight)
{
	if (!_DirValid(dir)) return STATUS_ARG_ERR;
	HAL_ENG_Enable(true);

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
 	HAL_ENG_Enable(true);

	if (angle > 90)//steps_to_do = angle * PI * 2 * wheel_distance / ( 2 * 180 * circumfirance_of_wheel / 6_calibarting_points)
 	{
 		distance = distance - wheelSafety[ED_LEFT][0] * _wheelDia * PI_CONST/_encRes;
 		angle-= 90;
 		if (smallRadius == 0.0) smallRadius = (distance/sin(angle*PI_CONST/180) - _wheelSpacing/2);
 		wheelCounter[ED_LEFT] = lroundf((smallRadius * (angle*PI_CONST)/180) * _encRes/(PI_CONST*_wheelDia));
 		wheelCounter[ED_RIGHT] = lroundf(((smallRadius + _wheelSpacing)*(angle*PI_CONST)/180) * _encRes/(PI_CONST*_wheelDia));
 		if (wheelCounter[ED_LEFT] < wheelSafety[ED_LEFT][0])
 		{
 			wheelCounter[ED_RIGHT] = wheelCounter[ED_RIGHT] - wheelCounter[ED_LEFT] + wheelSafety[ED_LEFT][0];
 			wheelCounter[ED_LEFT] = wheelSafety[ED_LEFT][0];
 		}
 		else
 		{
 			wheelSafety[ED_LEFT][0] = wheelCounter[ED_LEFT] - wheelSafety[ED_LEFT][0];
 			wheelSafety[ED_RIGHT][0] = 0;
 		}
 		//Neglect safety for now.. TODO: feature
 		safetyCounter[ED_LEFT] = 0;
 		safetyCounter[ED_RIGHT] = 0;
 		speedFactor = (float)wheelCounter[ED_LEFT]/((float)wheelCounter[ED_RIGHT]);

 		HAL_ENG_SetPWM(ED_RIGHT, ENGINE_FULL);	//set right engine speed
 		HAL_ENG_SetPWM(ED_LEFT, ENGINE_FULL * speedFactor * 0.9);	//set left engine speed
 	}
 	else
 	{
 		distance = distance - wheelSafety[ED_RIGHT][0] * _wheelDia * PI_CONST/_encRes;
 		angle = 90 - angle;
 		if (smallRadius == 0.0) smallRadius = (distance/sin(angle*PI_CONST/180) - _wheelSpacing/2);
 		wheelCounter[ED_RIGHT] = lroundf((smallRadius * (angle*PI_CONST)/180)*_encRes/(PI_CONST*_wheelDia));
 		wheelCounter[ED_LEFT] = lroundf(((smallRadius + _wheelSpacing)*(angle*PI_CONST)/180)*_encRes/(PI_CONST*_wheelDia));
 		if (wheelCounter[ED_RIGHT] < wheelSafety[ED_RIGHT][0])
 		{
 			wheelCounter[ED_LEFT] = wheelCounter[ED_LEFT] - wheelCounter[ED_RIGHT] + wheelSafety[ED_RIGHT][0];
 			wheelCounter[ED_RIGHT] = wheelSafety[ED_RIGHT][0];
 		}
 		else
 		{
 			wheelSafety[ED_RIGHT][0]= wheelCounter[ED_RIGHT] - wheelSafety[ED_RIGHT][0];
 			wheelSafety[ED_LEFT][0] = 0;
 		}
 		safetyCounter[ED_LEFT] = 0;
 		safetyCounter[ED_RIGHT] = 0;
 		speedFactor = (float)wheelCounter[ED_RIGHT]/((float)wheelCounter[ED_LEFT]);	//Speed difference between the engines



 		HAL_ENG_SetPWM(ED_LEFT, ENGINE_FULL);	//set right engine speed
 		HAL_ENG_SetPWM(ED_RIGHT, ENGINE_FULL * speedFactor *0.9);	//set left engine speed
 	}
	wheelCounter[ED_LEFT] -= engineOffset;
	HAL_ENG_SetHBridge(ED_BOTH, DIRECTION_FORWARD);

    HAL_ENG_IntEnable(ED_LEFT, true);
    HAL_ENG_IntEnable(ED_RIGHT, true);

	while ( IsDriving() );
	HAL_ENG_Enable(false);

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
 * Set values for safety of a specified sequence and update safety counter
 */
void EngineData::SetSafetySeq(uint8_t seq, uint32_t right, uint32_t left)
{
	wheelSafety[ED_LEFT][seq]= right;
	wheelSafety[ED_RIGHT][seq] = left;

	safetyCounter[ED_LEFT] = seq;
	safetyCounter[ED_RIGHT] = seq;
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


