/**
 * radarGP2.c
 *
 *  Created on: 29. 5. 2016.
 *      Author: Vedran
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "../libs/myLib.h"
#include "radarGP2.h"
#include "../HAL/hal.h"
#include "utils/uartstdio.h"

//  Global pointer to FIRST created instance of RadarModule
RadarModule* __rD;

void _RADAR_KernelCallback(void)
{
    //  Check for null-pointer
    if (__rD->_radKer.args == 0)
        return;

    /*
     *  Data in args[] contains bytes that constitute arguments for function
     *  calls. The exact representation(i.e. whether bytes represent ints, floats)
     *  of data is known only to individual blocks of switch() function. There
     *  is no predefined data separator between arguments inside args[].
     */
    switch(__rD->_radKer.serviceID)
    {
    /*
     * Request a radar scan by rotating horizontal axis from 0� to 160�. First
     * data byte is either 1(fine scan) or 0(coarse scan).
     * args[] = scanType(1B)
     * retVal none
     */
    case RADAR_SCAN:
        {
            //  Double negation to convert any integer into boolean
            bool fine = !(!__rD->_radKer.args[0]);
            __rD->Scan(fine);
        }
        break;
        /*
         * Rotate radar horizontally to a specified angle (0�-right, 160�-left)
         * args[] = angle(4B)
         * retVal none
         */
    case RADAR_SETH:
        {
            //  Only allowed to have 4 bytes of data (float)
            if (__rD->_radKer.argN == sizeof(float))
            {
                float angle;
                //  Copy data into a float
                memcpy((void*)&angle,
                       (void*)( __rD->_radKer.args),
                       sizeof(float));
                __rD->SetHorAngle(angle);
            }
        }
        break;
        /*
         * Rotate radar vertically to a specified angle (0�-up, 160�-down)
         * args[] = angle(4B)
         * retVal none
         */
    case RADAR_SETV:
        {
            //  Only allowed to have 4 bytes of data (float)
            if (__rD->_radKer.argN == sizeof(float))
            {
                float angle;
                //  Copy data into a float
                memcpy((void*)&angle,
                       (void*)( __rD->_radKer.args),
                       sizeof(float));
                __rD->SetVerAngle(angle);
            }
        }
        break;
    default:
        break;
    }

}

/*******************************************************************************
 *******************************************************************************
 *********            RadarModule class member functions                *********
 *******************************************************************************
 ******************************************************************************/

///-----------------------------------------------------------------------------
///                      Class constructor & destructor                 [PUBLIC]
///-----------------------------------------------------------------------------

RadarModule::RadarModule() : _scanComplete(false), _scanData(0), _fineScan(false)
{
    if (__rD == 0)
        __rD = this;
}

RadarModule::~RadarModule()
{
	__rD = 0;
}

///-----------------------------------------------------------------------------
///                      Class member function definitions              [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Initialize hardware used for radar and (if using task scheduler) register
 * kernel module so TS can make calls to this library.
 */
void RadarModule::InitHW()
{
    HAL_RAD_Init();

    //SetHorAngle(0);
    SetVerAngle(100);

#if defined(__USE_TASK_SCHEDULER__)
    //  Register module services with task scheduler
    _radKer.callBackFunc = _RADAR_KernelCallback;
    TS_RegCallback(&_radKer, RADAR_UID);
#endif
}

/**
 * Register hook to user function
 * Register hook to user-function called every time a scan is completed
 * @param funPoint pointer to a user function taking two arguments
 * (scan data buffer and its size)
 */
void RadarModule::AddHook(void((*funPoint)(uint8_t*, uint16_t*)))
{
    custHook = funPoint;
}
/**
 * Perform scan with radar and pass the results into the argument array
 * @param data pointer to array of min 160 sensor measurements
 * 				 if [fine] is true data has to be at least 1280 bytes long
 * @param length size of the array upon finalizing radar scan
 * @param fine if true scan results will not be averaged but all 1280 points
 * 				 will be saved in [data] array
 */
void RadarModule::Scan(uint8_t *data, uint16_t *length, bool fine)
{
	/*	Step corresponds to 1/8� making total of ~1270 point @ 160� area
	 * 		alternatively step=36.25 for 1� (making ~160 points @ 160� area)
	 */
	float angle = 0,
	      step = 0.125f;
	uint32_t dist, 		    //  Distance in cm, value returned from ADC module
			 angleAvg = 0,	//Temp. variable to calculate average of 8 readings
			 angleCount = 0;//counts number of measurements

	//  Enable PWM for radar
	HAL_RAD_Enable(true);

	//  Start at ~10� horizontal angle(far right)
	if (HAL_RAD_GetHorAngle() != 0)
	    HAL_RAD_SetHorAngle(0);

	//  Start at ~100� vertical angle(roughly middle)
	if (HAL_RAD_GetVerAngle() != 100)
	        HAL_RAD_SetVerAngle(100);

	//  Time for sensor to position itself to starting point
	HAL_DelayUS(30000);

	/*
	 * Start scan with radar turned to the right (~10�), scan from right to
	 * 	left, with resolution of 8 measurements within a single degree of
	 * 	rotation. If fine scan is requested, all measurements are pushed into a
	 * 	[data] array. If not 8 measurements are averaged together giving one
	 * 	measurement for each degree of rotation.
	 * 	7ms settling time between 0.125� -> avg speed of scanning 18�/s
	 */
	while (angle <= 160.0f)
	{
		//  Change the angle of radar
	    HAL_RAD_SetHorAngle(angle);
	    HAL_DelayUS(7000);	//  Settling time

		//  Trigger AD conversion, wait for data-ready flag and read data
	    dist = HAL_RAD_ADCTrigger();

	    //  Convert ADC readout to cm (according to datasheet graph)
	    if ( dist>2860 ) dist=10;
	    else if ( (dist<=2860) && (dist>2020) )
	    	dist = interpolate(2860,10,2020,15,dist);
	    else if ( (dist<=2020) && (dist>1610) )
	    	dist = interpolate(2020,15,1610,20,dist);
	    else if ( (dist<=1610) && (dist>1340) )
	    	dist = interpolate(1610,20,1340,25,dist);
	    else if ( (dist<=1340) && (dist>1140) )
	    	dist = interpolate(1340,25,1140,30,dist);
	    else if ( (dist<=1140) && (dist>910) )
	    	dist = interpolate(1140,30,910,40,dist);
	    else if ( (dist<=910) && (dist>757) )
	    	dist = interpolate(910,40,757,50,dist);
	    else if ( (dist<=757) && (dist>640) )
	    	dist = interpolate(757,50,640,60,dist);
	    else if ( (dist<=640) && (dist>540) )
	    	dist = interpolate(640,60,540,70,dist);
	    else if ( (dist<=540) && (dist>508) )
	    	dist = interpolate(540,70,508,80,dist);
	    else if ( (dist<=508)) dist=80;

	    //  Sum current distance with previous, to calculate average later
	    angleAvg += dist;
	    angleCount++;	//  Increase measurement counter

	    //  Check if the distance is to be saved to array
	    if (!fine && ((angleCount % 8) == 0))
	    {
	    	data[(angleCount/8)-1] = (angleAvg / 8) & 0xFF;
	    	angleAvg = 0;
	    }
	    else if (fine) data[angleCount] = dist;

	    angle += step;	//  Move radar to the next measurement point
	}

	//  Update length argument
	if (!fine) *length = angleCount / 8;
	else *length = angleCount;

	_scanComplete = true;
}

/**
 * Perform radar scan but provide internal buffer to save the data to.
 * @param fine specifies whether a fine scan (1280 points) is requested or
 * a coarse scan (160 points)
 */
void RadarModule::Scan(bool fine)
{
    uint16_t scanLen = 0;
    //uint8_t data[1280];

    //  If there's memory already occupied, it's assumed that's from old scan
    //  and can be safely deleted -> new memory will be occupied for this scan
    if (_scanData != 0)
        delete [] _scanData;

    //  Reserve memory space to fit measurements
    if (fine)
        _scanData = new uint8_t[1282];
    else
        _scanData = new uint8_t[162];

    //  Initiate scan
    Scan(_scanData, &scanLen, fine);
    //  Call user's function to process data from the scan
    custHook(_scanData, &scanLen);

    /*
     * Delete function on [_scanData] is not called here as this data might be
     * used again before the next scan. Rather, data buffered is cleared at the
     * beginning of this function.
     */
}

/**
 * Set horizontal angle of radar to a specified value (0� right, 160� left)
 * @param angle
 */
void RadarModule::SetHorAngle(float angle)
{
    HAL_RAD_SetHorAngle(angle); //  Direct call to HAL
}

/**
 * Set vertical angle of radar to a specified value (0� up, 160� down)
 * @param angle
 */
void RadarModule::SetVerAngle(float angle)
{
    HAL_RAD_SetVerAngle(angle); //  Direct call to HAL
}
