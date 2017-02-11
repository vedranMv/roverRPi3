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

    if ((HAL_ENG_GetHBridge(ED_BOTH) & 0x03) == DIR_WHEEL_BCK)
        __ed->wheelCounter[ED_LEFT]--;
    else if ((HAL_ENG_GetHBridge(ED_BOTH) & 0x03) == DIR_WHEEL_FWD)
        __ed->wheelCounter[ED_LEFT]++;

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

    if ((HAL_ENG_GetHBridge(ED_BOTH) >> 2) == DIR_WHEEL_BCK)
        __ed->wheelCounter[ED_RIGHT]--;
    else if ((HAL_ENG_GetHBridge(ED_BOTH) >> 2) == DIR_WHEEL_FWD)
        __ed->wheelCounter[ED_RIGHT]++;
}

/**
 * Wheel control loop - simple P controller on both wheels that calculates PWM
 * based on the error in position
 * If result for speed of control loop on distance produces speed higher than
 * max speed change to applying speed control loop, otherwise use speed from
 * position control
 * for dt =0.05 -> Kp=.18 Ki=0.52
 */
void ControlLoop(void)  //ISR
{
    static int32_t old[2] = {0}, zeros = 0;
    static float corrLocal[2] = {0.1, 0.1}, posI[2] = {0};//0.06
    static const float dT = 0.1f, Kp[2]={0.11, 0.05}, Ki[2] = {0.3, 0.38};//0.4

    GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_2, 0xFF);

    // Control loop for wheels
    for (uint8_t i = ED_LEFT; i <= ED_RIGHT; i++)
    {
        //  Calculate current speed of the wheel, dt=0.1
        __ed->speedCurr[i] = (float)(__ed->wheelCounter[i]-old[i]); //dPoints
        //  dAlpha=dPoints*360/encResolution;[�(deg)]
        __ed->speedCurr[i] = __ed->speedCurr[i] * 360.0f / __ed->_encRes;
        //  dist=wheelDia*pi*dAlpha/360;[cm]
        __ed->speedCurr[i] = __ed->_wheelDia*PI_CONST*__ed->speedCurr[i]/360.0f;
        //  speed=dist/dt;[cm/s]
        __ed->speedCurr[i] = __ed->speedCurr[i]/dT;

        //  Calculate error and controller output
        float error = __ed->wheelSetpoint[i] - __ed->wheelCounter[i];
        if (error < 6)
            posI[i] += error*dT;
        else posI[i] = 0;

        if (fabsf(error) < 2.0f)
        {
            zeros++;
            continue;
        }

        //  Only P controller -> correction = Kp*e
        float correction = Kp[i]*error + Ki[i]*posI[i];//0,13P0,1I

        UARTprintf("%d: %d__%d\n", i, lroundf(error), lroundf(__ed->speedCurr[i]));
        //  If current speed is below speed limit use position PID control
        if (fabsf(__ed->speedCurr[i]) < __ed->speedSetpoint[i])
        {
            //  Adjust direction of rotation (in case position is overshot)
            if (correction > 0)
                HAL_ENG_SetHBridge(i, DIR_WHEEL_FWD<<(2*i));
            else
                HAL_ENG_SetHBridge(i, DIR_WHEEL_BCK<<(2*i));
            //  Save newly set correction
            corrLocal[i] = fabsf(correction);
        }
        //  If current speed is over the speed limit but newly calculated speed
        //  is slower update current speed
        else if (corrLocal[i] > correction)
        {
            //  Adjust direction of rotation (in case position is overshot)
            if (correction > 0)
                HAL_ENG_SetHBridge(i, DIR_WHEEL_FWD<<(2*i));
            else
                HAL_ENG_SetHBridge(i, DIR_WHEEL_BCK<<(2*i));
            //  Save newly set correction
            corrLocal[i] = fabsf(correction);
        }
        //  Else keep current speed

        //  Update PWM
        uint32_t tmp = (uint32_t)((float)ENGINE_FULL*corrLocal[i])+1;
        HAL_ENG_SetPWM(i, (tmp<ENGINE_FULL?tmp:ENGINE_FULL));

        //  Count number of zero-errors in order to find steady-state
        if (lroundf(error) == 0) zeros++;
        else zeros = 0;

        //  Save values for next run
        old[i] = __ed->wheelCounter[i];
    }
    //  Keep at the end of ISR as it also restarts the timer
    HAL_ENG_TimIntClear(true);
    if (zeros >= 6)
    {
        zeros = 0;
        old[0] = old[1] = 0;
        corrLocal[0] = corrLocal[1] = 0.2;
        posI[0] = posI[1] = 0;

        HAL_ENG_Enable(false);
        HAL_ENG_TimControl(false);
    }
    GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_2, 0x00);

}

int8_t EngineData::InitHW()
{
    HAL_ENG_Init(ENGINE_STOP, ENGINE_FULL);
    //  Run control loop 10Hz 50->20Hz
    HAL_ENG_TimInit(100, ControlLoop);
    //  Listen for encoder input
    HAL_ENG_IntEnable(ED_LEFT, true);
    HAL_ENG_IntEnable(ED_RIGHT, true);
    //HAL_ENG_TimControl(true);
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

	HAL_ENG_Enable(true);
	HAL_ENG_TimControl(true);
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

 	wheelSetpoint[ED_LEFT] = lroundf(wheelDistance);

 	wheelSetpoint[ED_RIGHT] = wheelSetpoint[ED_LEFT];	//set counter for right engine

	//if any wheel engine is turned off, don't count for it
	if ( (dir & 0x03) == 0 ) wheelSetpoint[ED_LEFT] = 0;
	if ( ((dir >> 2) & 0x03) == 0 ) wheelSetpoint[ED_RIGHT] = 0;

	//  Check direction of wheel and whether to either add or subtract distance
    if ((dir & 0x0C) == 4)
        wheelSetpoint[ED_RIGHT] += wheelCounter[ED_RIGHT];
    else if ((dir & 0x0C) == 8)
        wheelSetpoint[ED_RIGHT] -= wheelCounter[ED_RIGHT];
    if ((dir & 0x03) == 1)
        wheelSetpoint[ED_LEFT] += wheelCounter[ED_LEFT];
    else if ((dir & 0x03) == 2)
        wheelSetpoint[ED_LEFT] -= wheelCounter[ED_LEFT];

#if defined(__DEBUG_SESSION__)
	UARTprintf("Going %d LEFT: %d   RIGHT: %d  \n", dir, wheelCounter[ED_LEFT], wheelCounter[ED_RIGHT]);
#endif
	/*HAL_ENG_SetPWM(ED_LEFT, ENGINE_FULL);	//set left engine speed
	HAL_ENG_SetPWM(ED_RIGHT, ENGINE_FULL);	//set right engine speed*/
	HAL_ENG_SetHBridge(ED_BOTH, dir);       //configure H-bridge

	//HAL_ENG_IntEnable(ED_RIGHT, true);

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


