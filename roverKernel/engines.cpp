/*
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


volatile EngineData* __ed;
int32_t engineOffset=0;	//Error in measurements between optical encoders - 0


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
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, ~GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0));

	__ed->wheelCounter[ED_LEFT]--;
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

	__ed->wheelCounter[ED_RIGHT]--;
}

/**
 * Do control on both speed and position
 * If result for speed of control loop on distance produces speed higher than
 * max speed change to applying speed control loop, otherwise use speed from
 * position control
 */
void ControlLoop(void)  //ISR
{
    static int32_t oldL = 0, oldR = 0;
    static const uint8_t threshold = 5;
    static float posI[2] = {0}, speedI[2] = {0};

    //  Adjust PWM +/-5 is threshold for activating control loop
    if (labs(__ed->wheelCounter[ED_LEFT]) < threshold)
        HAL_ENG_SetPWM(ED_LEFT, ENGINE_STOP);
    else if (labs(__ed->wheelCounter[ED_LEFT]) >= threshold)
    {// Control loop for left wheel
        //  Calculate current speed of left wheel, dt=0.1
        float tmpSpeed = (float)(oldL-__ed->wheelCounter[ED_LEFT]); //dPoints
        //  dAlpha=dPoints*360/encResolution;[°(deg)]
        tmpSpeed = tmpSpeed * 360.0f / __ed->_encRes;
        //  dist=wheelDia*pi*dAlpha/360;[cm]
        tmpSpeed = __ed->_wheelDia*PI_CONST*tmpSpeed/360;
        //  speed=dist/dt;[cm/s]
        tmpSpeed = tmpSpeed/0.1f;

        float error = __ed->speed[ED_LEFT]-tmpSpeed;
        speedI[ED_LEFT]+=error;

        //  correction=Kp*e+Ki*I+Kd*d
        float correction = (0.7)*error + \
                           (10)*speedI[ED_LEFT] + \
                           (3)*((float)(oldL-__ed->wheelCounter[ED_LEFT]));
    }

    if (labs(__ed->wheelCounter[ED_RIGHT]) < threshold)
        HAL_ENG_SetPWM(ED_RIGHT, ENGINE_STOP);
    else if (labs(__ed->wheelCounter[ED_RIGHT]) >= threshold)
    {// Control loop for right wheel

    }


    //  Save values for next run
    oldL = __ed->wheelCounter[ED_LEFT];
    oldR = __ed->wheelCounter[ED_RIGHT];
    //  Keep at the end of ISR as it restarts the timer
    HAL_ENG_TimIntClear(true);
}

int8_t EngineData::InitHW()
{
    HAL_ENG_Init(ENGINE_STOP, ENGINE_FULL);
    //  Run control loop 10Hz
    HAL_ENG_TimInit(100, ControlLoop);

	return STATUS_OK;
}

/**
 * Move vehicle in desired direction
 * 		@param direction - selects the direction of movement
 * 		@param arg - distance in centimeters(forward/backward) or angle in °(left/right)
 * 	TODO: Configure startup_ccs.c to support ISR for counters
 */
int8_t EngineData::StartEngines(uint8_t dir, float arg, bool blocking)
{
	float wheelDistance ; //centimeters

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
#if defined(__DEBUG_SESSION__)
	UARTprintf("Going %d LEFT: %d   RIGHT: %d  \n", dir, wheelCounter[ED_LEFT], wheelCounter[ED_RIGHT]);
#endif
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
 * 		@param angle - angle in °(left/right) that's needed to travel along the arc
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
 * Check if the passed arguments are valid
 */
bool EngineData::_DirValid(uint8_t dir)
{
	if ((dir == DIRECTION_BACKWARD) || (dir == DIRECTION_FORWARD)
			|| (dir == DIRECTION_LEFT) || (dir == DIRECTION_RIGHT))
		return true;
	return false;
}


