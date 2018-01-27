/**
 * mpu9250.cpp
 *
 *  Created on: 25. 3. 2015.
 *      Author: Vedran
 */
#include "mpu9250.h"

#if defined(__HAL_USE_MPU9250_NODMP__)       //  Compile only if module is enabled

#include "HAL/hal.h"
#include "api_mpu9250.h"
#include "libs/myLib.h"
#include "libs/helper_3dmath.h"


//  Enable debug information printed on serial port
#define __DEBUG_SESSION__


//  Integration with event log, if it's present
#ifdef __HAL_USE_EVENTLOG__
    #include "init/eventLog.h"
    //  Simplify emitting events
    #define EMIT_EV(X, Y)  EventLog::EmitEvent(MPU_UID, X, Y)
#endif  /* __HAL_USE_EVENTLOG__ */

#ifdef __DEBUG_SESSION__
#include "serialPort/uartHW.h"

#define _FTOI_(X) (int16_t)(trunc(X)),(int16_t)(trunc((X-trunc(X))*100))
#endif


#if defined(__USE_TASK_SCHEDULER__)

/**
 * Callback routine to invoke service offered by this module from task scheduler
 * @note It is assumed that once this function is called task scheduler has
 * already copied required variables into the memory space provided for it.
 */
void _MPU_KernelCallback(void)
{
    MPU9250 &__mpu = MPU9250::GetI();

    static float sumOfRot = 0;


    //  Check for null-pointer
    if (__mpu._mpuKer.args == 0)
        return;

    /*
     *  Data in args[] contains bytes that constitute arguments for function
     *  calls. The exact representation(i.e. whether bytes represent ints, floats)
     *  of data is known only to individual blocks of switch() function. There
     *  is no predefined data separator between arguments inside args[].
     */
    switch (__mpu._mpuKer.serviceID)
    {
    /*
     * Change state of the power switch. Allows for powering down MPU chip
     * args[] = powerState(bool)
     * retVal one of MPU_* error codes
     */
    case MPU_T_POWERSW:
        {
            //  Double negation to convert any non-zero int to bool
            bool powerState = !(!(__mpu._mpuKer.args[0]));

            HAL_MPU_PowerSwitch(powerState);

            //  If sensor is powering on reset I2C and load DMP firmware
            if (powerState)
            {
                __mpu.InitHW();
                __mpu.InitSW();
            }
            __mpu._mpuKer.retVal = MPU_SUCCESS;
        }
        break;

    /*
     *  Once new data is available, read it from FIFO and store in data structure
     *  args[] = none
     *  retVal one of MPU_* error codes
     */
    case MPU_T_GET_DATA:
        {
            static bool suppressError = false;

            //if (HAL_MPU_DataAvail())
            {
                static int16_t gyro[3], accel[3], mag[3];


            #ifdef __USE_TASK_SCHEDULER__
                //  Calculate dT in seconds!
                static uint64_t oldms = 0;
                __mpu.dT = (float)(msSinceStartup-oldms)/1000.0f;
                oldms = msSinceStartup;
            #endif /* __HAL_USE_TASKSCH__ */

                readAccelData(accel);
                readGyroData(gyro);
                readMagData(mag);

                for (uint8_t i = 0; i < 3; i++)
                {
                    __mpu._acc[i] = (float)accel[i] * getAres();
                    __mpu._gyro[i] = (float)gyro[i] * getGres();
                    __mpu._mag[i] = (float)mag[i] * getMres();
                }
//                for (uint8_t i = 0; i < 128; i++)
//                    DEBUG_WRITE("\x08");
                //DEBUG_WRITE("Accel: %02d.%03dX  %02d.%03dY  %02d.%03dZ\n", _FTOI_(__mpu._acc[0]), _FTOI_(__mpu._acc[1]), _FTOI_(__mpu._acc[2]));
                //DEBUG_WRITE("Gyro: %02d.%03dX  %02d.%03dY  %02d.%03dZ\n", _FTOI_(__mpu._gyro[0]), _FTOI_(__mpu._gyro[1]), _FTOI_(__mpu._gyro[2]));
                DEBUG_WRITE("MAg: %02d.%03dX  %02d.%03dY  %02d.%03dZ\n", _FTOI_(__mpu._mag[0]), _FTOI_(__mpu._mag[1]), _FTOI_(__mpu._mag[2]));

                //DEBUG_WRITE("Mag: %02X  %02X %02X \n", mag[0], mag[1], mag[2]);
            }

        }
        break;
        /*
         * Restart MPU module and reload DMP firmware
         * args[] = rebootCode(0x17)
         * retVal one of MPU_* error codes
         */
    case MPU_T_REBOOT:
        {
            if (__mpu._mpuKer.args[0] == 0x17)
            {
                sumOfRot = 0.0; //Prevents error for big change in value after reboot
                __mpu.Reset();
                __mpu.InitHW();
                __mpu._mpuKer.retVal = (int32_t)__mpu.InitSW();
            }
        }
        break;
        /*
         * Soft reboot of MPU -> only reset status in event logger
         * args[] = rebootCode(0x17)
         * retVal one of MPU_* error codes
         */
    case MPU_T_SOFT_REBOOT:
        {
            if (__mpu._mpuKer.args[0] == 0x17)
            {
#ifdef __HAL_USE_EVENTLOG__
                EventLog::SoftReboot(MPU_UID);
#endif  /* __HAL_USE_EVENTLOG__ */
            }
        }
        break;
    default:
        break;
    }

    //  Report outcome to event logger
#ifdef __HAL_USE_EVENTLOG__
    if (__mpu._mpuKer.retVal == MPU_SUCCESS)
        EMIT_EV(__mpu._mpuKer.serviceID, EVENT_OK);
    else
        EMIT_EV(__mpu._mpuKer.serviceID, EVENT_ERROR);
#endif  /* __HAL_USE_EVENTLOG__ */
}
#endif

/*******************************************************************************
 *******************************************************************************
 *********              MPU9250 class member functions                 *********
 *******************************************************************************
 ******************************************************************************/

///-----------------------------------------------------------------------------
///         Functions for returning static instance                     [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Return reference to a singleton
 * @return reference to an internal static instance
 */
MPU9250& MPU9250::GetI()
{
    static MPU9250 singletonInstance;
    return singletonInstance;
}

/**
 * Return pointer to a singleton
 * @return pointer to a internal static instance
 */
MPU9250* MPU9250::GetP()
{
    return &(MPU9250::GetI());
}

///-----------------------------------------------------------------------------
///         Public functions used for configuring MPU9250               [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Initialize hardware used by MPU9250
 * Initializes I2C bus for communication with MPU (SDA - PN4, SCL - PN5), bus
 * frequency 1MHz, connection timeout: 100ms. Initializes pin(PA5)
 * to be toggled by MPU9250 when it has data available for reading (PA5 is
 * push-pull pin with weak pull down and 10mA strength).
 * @return One of MPU_* error codes
 */
int8_t MPU9250::InitHW()
{
    HAL_MPU_Init();
    HAL_MPU_PowerSwitch(true);

    //  Emit event before initializing module
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_STARTUP);
#endif  /* __HAL_USE_EVENTLOG__ */

#if defined(__USE_TASK_SCHEDULER__)
    //  Register module services with task scheduler
    _mpuKer.callBackFunc = _MPU_KernelCallback;
    TS_RegCallback(&_mpuKer, MPU_UID);
#endif

    return MPU_SUCCESS;
}

/**
 * Initialize MPU sensor, load DMP firmware and configure DMP output. Prior to
 * any software initialization, this function power-cycles the board
 * @return One of MPU_* error codes
 */
int8_t MPU9250::InitSW()
{
    //  Power cycle MPU chip on every SW initialization
    HAL_MPU_PowerSwitch(false);
    HAL_DelayUS(20000);
    HAL_MPU_PowerSwitch(true);
    HAL_DelayUS(30000);


    //  Emit event before initializing module
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_STARTUP);
#endif  /* __HAL_USE_EVENTLOG__ */

#ifdef __DEBUG_SESSION__
    DEBUG_WRITE("Starting up initialization\n");
#endif

    initMPU9250();
    initAK8963();

#ifdef __HAL_USE_EVENTLOG__
EMIT_EV(-1, EVENT_ERROR);
#endif  /* __HAL_USE_EVENTLOG__ */


#ifdef __DEBUG_SESSION__
    DEBUG_WRITE("done\n");
#endif

    //  Update status of initialization process
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_INITIALIZED);
#endif  /* __HAL_USE_EVENTLOG__ */

    return MPU_SUCCESS;
}

/**
 * Trigger software reset of the MPU module by writing into corresponding register
 */
void MPU9250::Reset()
{
    HAL_MPU_WriteByteNB(MPU9250_ADDRESS, PWR_MGMT_1, 1 << 7);
    HAL_DelayUS(50000);

#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_UNINITIALIZED);
#endif  /* __HAL_USE_EVENTLOG__ */

}

/**
 * Check if new sensor data has been received
 * @return true if new sensor data is available
 *        false otherwise
 */
bool MPU9250::IsDataReady()
{
    return static_cast<bool>(_dataFlag);
}

/**
 * Get ID from MPU, should always return 0x71
 * @return ID value stored in MPU's register
 */
uint8_t MPU9250::GetID()
{
    uint8_t ID;
    ID = HAL_MPU_ReadByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

    return ID;
}

/**
 * Add hook to user-defined function to be called once new sensor
 * data has been received
 * @param custHook pointer to function with two float args (accel & gyro array)
 */
void MPU9250::AddHook(void((*custHook)(uint8_t,float*)))
{
    userHook = custHook;
}

/**
 * Copy orientation from internal buffer to user-provided one
 * @param RPY pointer to float buffer of size 3 to hold roll-pitch-yaw
 * @param inDeg if true RPY returned in degrees, if false in radians
 */
void MPU9250::RPY(float* RPY, bool inDeg)
{
    for (uint8_t i = 0; i < 3; i++)
        if (inDeg)
            RPY[i] = _ypr[i]*180.0/PI_CONST;
        else
            RPY[i] = _ypr[i];
}

/**
 * Copy acceleration from internal buffer to user-provided one
 * @param acc Pointer a float array of min. size 3 to store 3-axis acceleration
 *  data
 */
void MPU9250::Acceleration(float *acc)
{
    memcpy((void*)acc, (void*)_acc, sizeof(float)*3);
}

///-----------------------------------------------------------------------------
///                      Class constructor & destructor              [PROTECTED]
///-----------------------------------------------------------------------------

MPU9250::MPU9250() :  dT(0), _dataFlag(false), userHook(0)
{
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_UNINITIALIZED);
#endif  /* __HAL_USE_EVENTLOG__ */
}

MPU9250::~MPU9250()
{}

#endif  /* __HAL_USE_MPU9250__ */
