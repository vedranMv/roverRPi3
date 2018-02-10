/**
 * radarGP2.c
 *
 *  Created on: 29. 5. 2016.
 *      Author: Vedran
 */
#include "radarGP2.h"

#if defined(__HAL_USE_RADAR__)       //  Compile only if module is enabled

#include "libs/myLib.h"
#include "HAL/hal.h"

//  Integration with event log, if it's present
#ifdef __HAL_USE_EVENTLOG__
    #include "init/eventLog.h"
    //  Simplify emitting events
    #define EMIT_EV(X, Y)  EventLog::EmitEvent(RADAR_UID, X, Y)
#endif  /* __HAL_USE_EVENTLOG__ */

#ifdef __DEBUG_SESSION__
#include "serialPort/uartHW.h"
#endif

#if defined(__USE_TASK_SCHEDULER__)
/**
 * Callback routine to invoke service offered by this module from task scheduler
 * @note It is assumed that once this function is called task scheduler has
 * already copied required variables into the memory space provided for it.
 */
void _RADAR_KernelCallback(void)
{
    RadarModule &__rD = RadarModule::GetI();
    //  Check for null-pointer
    if (__rD._ker.args == 0)
        return;

    /*
     *  Data in args[] contains bytes that constitute arguments for function
     *  calls. The exact representation(i.e. whether bytes represent ints, floats)
     *  of data is known only to individual blocks of switch() function. There
     *  is no predefined data separator between arguments inside args[].
     */
    switch(__rD._ker.serviceID)
    {
    /*
     * Request a radar scan by rotating horizontal axis from 0� to 160�. First
     * data byte is either 1(fine scan) or 0(coarse scan).
     * args[] = none
     * retVal one of myLib.h STATUS_* error codes
     */
    case RADAR_T_SCAN:
        {
            static float horAngle = 0.0;
            static uint16_t scanLen = 0;

            if (horAngle < 160)
            {
                uint32_t dist;

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

                __rD._scanData[scanLen] = (uint8_t)(dist & 0xFF);

                scanLen++;
                horAngle+=1.0;
            }

            if (horAngle < 160.0)
            {
                HAL_RAD_SetHorAngle(horAngle);
                return; //  Return so we don't emit any event
            }
            else
            {
                __rD.custHook(__rD._scanData, &scanLen);
                __rD._scanComplete = true;
                horAngle = 0.0;
                scanLen = 0;

                //  Return radar to starting position after completing the scan
                HAL_RAD_SetHorAngle(horAngle);
                __rD._ker.retVal = STATUS_OK;
            }
        }
        break;
        /*
         * Rotate radar horizontally to a specified angle (0�-right, 160�-left)
         * args[] = angle(4B)
         * retVal none
         */
    case RADAR_T_SETH:
        {
            //  Only allowed to have 4 bytes of data (float)
            if (__rD._ker.argN == sizeof(float))
            {
                float angle;
                //  Copy data into a float
                memcpy((void*)&angle,
                       (void*)( __rD._ker.args),
                       sizeof(float));
                __rD.SetHorAngle(angle);
            }
        }
        break;
        /*
         * Rotate radar vertically to a specified angle (0�-up, 160�-down)
         * args[] = angle(4B)
         * retVal none
         */
    case RADAR_T_SETV:
        {
            //  Only allowed to have 4 bytes of data (float)
            if (__rD._ker.argN == sizeof(float))
            {
                float angle;
                //  Copy data into a float
                memcpy((void*)&angle,
                       (void*)( __rD._ker.args),
                       sizeof(float));
                __rD.SetVerAngle(angle);
            }
        }
        break;
        /*
         * Measures current distance and sets new angle
         * args[] = none
         * retVal one of myLib.h STATUS_* error codes
         */
    case RADAR_T_BLOCKINGSCAN:
    {
            __rD._ker.retVal = __rD.Scan(true);
    }
        break;
    default:
        break;
    }

    //  Check return-value and emit event based on it
#ifdef __HAL_USE_EVENTLOG__
    if (__rD._ker.retVal == STATUS_OK)
        EMIT_EV(__rD._ker.serviceID, EVENT_OK);
    else
        EMIT_EV(__rD._ker.serviceID, EVENT_ERROR);
#endif  /* __HAL_USE_EVENTLOG__ */
}
#endif /* __USE_TASK_SCHEDULER__ */

///-----------------------------------------------------------------------------
///         Functions for returning static instance                     [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Return reference to a singleton
 * @return reference to an internal static instance
 */
RadarModule& RadarModule::GetI()
{
    static RadarModule singletonInstance;
    return singletonInstance;
}

/**
 * Return pointer to a singleton
 * @return pointer to a internal static instance
 */
RadarModule* RadarModule::GetP()
{
    return &(RadarModule::GetI());
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
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_STARTUP);
#endif  /* __HAL_USE_EVENTLOG__ */
    HAL_RAD_Init();

    //SetHorAngle(0);
    SetVerAngle(100);
    //  Reserve memory space to fit measurements
    _scanData = new uint8_t[162];

#if defined(__USE_TASK_SCHEDULER__)
    //  Register module services with task scheduler
    _ker.callBackFunc = _RADAR_KernelCallback;
    TS_RegCallback(&_ker, RADAR_UID);
#endif

#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_INITIALIZED);
#endif  /* __HAL_USE_EVENTLOG__ */
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
 * @return error-code, one of STATUS_* macros from myLib.h
 */
uint32_t RadarModule::Scan(uint8_t *data, uint16_t *length)
{
	/*	Step corresponds to 1/8� making total of ~1270 point @ 160� area
	 * 		alternatively step=36.25 for 1� (making ~160 points @ 160� area)
	 */
	float angle = 0,
	      step = 0.125f;
	uint32_t dist, 		    //  Distance in cm, value returned from ADC module
			 angleAvg = 0,	//  Temp. variable to calculate average of 8 readings
			 angleCount = 0;//  Counts number of measurements

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
	    HAL_DelayUS(40000);	//  Settling time 40ms/1deg

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
	    if ((angleCount % 8) == 0)
	    {
	    	data[(angleCount/8)-1] = (angleAvg / 8) & 0xFF;
	    	angleAvg = 0;
	    }

	    angle += step;	//  Move radar to the next measurement point
	}

	//  Update length argument
	*length = angleCount / 8;

	//  Set the flag to indicate scan is completed
	_scanComplete = true;

	return STATUS_OK;
}

/**
 * Perform radar scan but provide internal buffer to save the data to.
 * @param fine specifies whether a fine scan (1280 points) is requested or
 * a coarse scan (160 points)
 * @param hook If true after scan passes scan data to a user-provided function
 * @return error-code, one of STATUS_* macros from myLib.h
 */
uint32_t RadarModule::Scan(bool hook)
{
    uint16_t scanLen = 0;
    uint32_t retVal;


    //  Initiate scan
    retVal = Scan(_scanData, &scanLen);

    //  Call user's function to process data from the scan
    if ((custHook != 0) && hook)
    {
        custHook(_scanData, &scanLen);

        //  After hooked function has processed data clear the flag
        _scanComplete = false;
    }

    return retVal;
}

/**
 * Check if the current scan is completed
 * @note Function clears internal _scanComplete flag after it has returned true!
 * @return whether or not the current scan is completed
 */
bool RadarModule::ScanReady()
{
    bool retVal = _scanComplete;
    _scanComplete = false;

    return retVal;
}

/**
 * Read whatever data is in the internal buffer (left from last completed scan)
 * @param buffer[out] buffer to which scan data is written into
 * @param bufferLen[out] length of the buffer above
 */
void RadarModule::ReadBuffer(uint8_t *buffer, uint16_t *bufferLen)
{
    if (_fineScan)
        *bufferLen = 1280;
    else
        *bufferLen = 160;

    memcpy((void*)buffer, (void*)_scanData, *bufferLen);
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

///-----------------------------------------------------------------------------
///                      Class constructor & destructor              [PROTECTED]
///-----------------------------------------------------------------------------

RadarModule::RadarModule() : _scanComplete(false), _scanData(0), _fineScan(false)
{
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_UNINITIALIZED);
#endif  /* __HAL_USE_EVENTLOG__ */
}

RadarModule::~RadarModule()
{}

#endif  /* __HAL_USE_RADAR__ */
