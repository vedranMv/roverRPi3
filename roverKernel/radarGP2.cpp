/*
 * radarGP2.c
 *
 *  Created on: 29. 5. 2016.
 *      Author: Vedran
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "radarGP2.h"
#include "myLib.h"
#include "tm4c1294_hal.h"

RadarData* __rD;

RadarData::RadarData() : _scanComplete(false)
{
	__rD = this;
}

RadarData::~RadarData()
{
	__rD = 0;
}

int8_t RadarData::InitHW()
{
    HAL_RAD_Init();

	return STATUS_OK;
}

/*
 * Perform scan with radar and pass the results into the argument array
 * \param data - pointer to array of min 160 sensor measurements
 * 				 if \param fine is true data has to be at least 1280 bytes long
 *	\param length - size of the array upon finalizing radar scan
 * \param fine - if true scan results will not be averaged but all 1280 points
 * 				 will be saved in \param data array
 * \return 0 if completed succesfully
 */
int8_t RadarData::Scan(uint8_t *data, uint16_t *length, bool fine)
{
	/*	Step corresponds to 1/8° making total of ~1270 point @ 160° area
	 * 		alternatively step=36.25 for 1° (making ~160 points @ 160° area)
	 */
	//float pwm = RADAR_RIGHT, step = 4.53125;
	float pwm = 0, step = 1;
	uint32_t dist, 		//holds distance in cm, value returned from ADC module
			 angleAvg = 0,	 //Temp variable to calcualte average of 8 readings
			 angleCount = 0; //counts number of measurements

	//  Enable PWM for radar
	HAL_RAD_Enable(true);

	//Start at ~10°resetRadar();
	if (HAL_RAD_GetHorAngle() != 0)
	    HAL_RAD_SetHorAngle(0);

	if (HAL_RAD_GetVerAngle() != 100)
	        HAL_RAD_SetVerAngle(100);

	//  Time for sensor to position itself to starting point
	HAL_DelayUS(500000);

	/*
	 * Start scan with radar turned to the right (~10°), scan from right to
	 * 	left, with resolution of 8 measurements within a single degree of
	 * 	rotation. However, 8 consecutive measurements will be average together
	 * 	for better precision, resulting in 160 measurements
	 */
	while (pwm <= 160.0f)
	{
		//  Change the angle of radar
	    //HAL_RAD_SetPWM(((uint32_t)pwm), RAD_HORIZONTAL );
	    HAL_RAD_SetHorAngle(pwm);
	    HAL_DelayUS(30000);	//  Settling time - 30ms

		//  Trigger AD conversion, wait for data-ready flag and read data
	    dist = HAL_RAD_ADCTrigger();

	    //  Convert ADC readout to cm (according to datahseet graph)
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
	    if (!fine && ((angleCount % 8) == 0) && (angleCount != 0))
	    {
	    	data[angleCount / 8] = (angleAvg / 8) & 0xFF;
	    	angleAvg = 0;
	    }
	    else if (fine) data[angleCount] = dist;

	    pwm += step;	//  Move radar to the next measurements point
	}

	//  Update length argument
	if (!fine) *length = angleCount / 8;
	else *length = angleCount;

	_scanComplete = true;
	//HAL_RAD_Enable(false);

	return STATUS_OK;	//  Successfull scan
}



