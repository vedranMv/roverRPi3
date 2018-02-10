/**
 *  mpu9250.cpp
 *
 *  Created on: 25. 3. 2015.
 *      Author: Vedran
 */
#include "mpu9250.h"

#if defined(__HAL_USE_MPU9250_NODMP__)  //  Compile only if module is enabled

#include "HAL/hal.h"
#include "api_mpu9250.h"
#include "libs/myLib.h"
#include "libs/helper_3dmath.h"

#include "registerMap.h"


//  Enable debug information printed on serial port
//#define __DEBUG_SESSION__


//  Integration with event log, if it's present
#ifdef __HAL_USE_EVENTLOG__
    #include "init/eventLog.h"
    //  Simplify emitting events
    #define EMIT_EV(X, Y)  EventLog::EmitEvent(MPU_UID, X, Y)
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
void _MPU_KernelCallback(void)
{
    MPU9250 &__mpu = MPU9250::GetI();

    static float sumOfRot = 0;


    //  Check for null-pointer
    if (__mpu._ker.args == 0)
        return;

    /*
     *  Data in args[] contains bytes that constitute arguments for function
     *  calls. The exact representation(i.e. whether bytes represent ints, floats)
     *  of data is known only to individual blocks of switch() function. There
     *  is no predefined data separator between arguments inside args[].
     */
    switch (__mpu._ker.serviceID)
    {
    /*
     * Change state of the power switch. Allows for powering down MPU chip
     * args[] = powerState(bool)
     * retVal one of MPU_* error codes
     */
    case MPU_T_POWERSW:
        {
            //  Double negation to convert any non-zero int to bool
            bool powerState = !(!(__mpu._ker.args[0]));

            HAL_MPU_PowerSwitch(powerState);

            //  If sensor is powering on reset I2C and load DMP firmware
            if (powerState)
            {
                __mpu.InitHW();
             //   __mpu.InitSW();
            }
            __mpu._ker.retVal = MPU_SUCCESS;
        }
        break;

    /*
     *  Once new data is available, read it from FIFO and store in data structure
     *  args[] = none
     *  retVal one of MPU_* error codes
     */
    case MPU_T_GET_DATA:
        {
            if (/*HAL_MPU_DataAvail()*/true)
            {
            #ifdef __USE_TASK_SCHEDULER__
                //  Calculate dT in seconds!
                static uint64_t oldms = 0;
                __mpu.dT = (float)(msSinceStartup-oldms)/1000.0f;
                oldms = msSinceStartup;
            #endif /* __HAL_USE_TASKSCH__ */

                __mpu.ReadSensorData();

#ifdef __DEBUG_SESSION__
                DEBUG_WRITE("{%02d.%03d, %02d.%03d, %02d.%03d, ", _FTOI_(__mpu._gyro[0]), _FTOI_(__mpu._gyro[1]), _FTOI_(__mpu._gyro[2]));
                DEBUG_WRITE("%02d.%03d, %02d.%03d, %02d.%03d, ", _FTOI_(__mpu._acc[0]), _FTOI_(__mpu._acc[1]), _FTOI_(__mpu._acc[2]));
                DEBUG_WRITE("%02d.%03d, %02d.%03d, %02d.%03d},\n", _FTOI_(__mpu._mag[0]), _FTOI_(__mpu._mag[1]), _FTOI_(__mpu._mag[2]));
#endif  /* __DEBUG_SESSION__ */
                if ((sumOfRot - (fabs(__mpu._ypr[0])+fabs(__mpu._ypr[1])+fabs(__mpu._ypr[2]))) > 30.0f)
                {
                    if (sumOfRot)
                    {
                #ifdef __HAL_USE_EVENTLOG__
                    EMIT_EV(__mpu._ker.serviceID, EVENT_HANG);
                #endif  /* __HAL_USE_EVENTLOG__ */
                    }
                }

                sumOfRot = fabs(__mpu._ypr[0])+fabs(__mpu._ypr[1])+fabs(__mpu._ypr[2]);
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
            if (__mpu._ker.args[0] == 0x17)
            {
                sumOfRot = 0.0; //Prevents error for big change in value after reboot
                __mpu.Reset();
                __mpu.InitHW();
                __mpu._ker.retVal = (int32_t)__mpu.InitSW();
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
            if (__mpu._ker.args[0] == 0x17)
            {
#ifdef __HAL_USE_EVENTLOG__
                EventLog::SoftReboot(MPU_UID);
#endif  /* __HAL_USE_EVENTLOG__ */
            }
        }
        break;
        /*
         * Change configuration of AHRS algorithm
         * args[] = kp(float)|ki(float)|magnetometerEnable(bool)
         * retVal one of MPU_* error codes
         */
    case MPU_T_AHRS_CONFIG:
        {
            float ki, kp;

            memcpy((void*)&kp, (void*)__mpu._ker.args, sizeof(float));
            memcpy((void*)&ki,
                   (void*)(__mpu._ker.args+sizeof(float)),
                   sizeof(float));

            //  Update magnetometer-enabled flag
            __mpu._magEn = (bool)*(__mpu._ker.args+2*sizeof(float));
            //  Update settings of the AHRS algorithm
            __mpu.SetupAHRS(0.0f, kp, ki);
        }
        break;
    default:
        break;
    }

    //  Report outcome to event logger
#ifdef __HAL_USE_EVENTLOG__
    if (__mpu._ker.retVal == MPU_SUCCESS)
        EMIT_EV(__mpu._ker.serviceID, EVENT_OK);
    else
        EMIT_EV(__mpu._ker.serviceID, EVENT_ERROR);
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
 * Initializes bus for communication with MPU, pin(PA5) to be toggled by MPU9250
 * when it has data available for reading.
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
    _ker.callBackFunc = _MPU_KernelCallback;
    TS_RegCallback(&_ker, MPU_UID);
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
    //  Power cycle MPU chip before every SW initialization
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
 * Trigger software reset of the MPU module by writing into corresponding
 * register. Wait for 50ms afterwards for sensor to start up.
 * @return One of MPU_* error codes
 */
int8_t MPU9250::Reset()
{
    HAL_MPU_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 1 << 7);
    HAL_DelayUS(50000);

#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_UNINITIALIZED);
#endif  /* __HAL_USE_EVENTLOG__ */

    return MPU_SUCCESS;
}

/**
 * Control power supply of the MPU9250
 * Enable or disable power supply of the MPU9250 using external MOSFET
 * @param en Power state
 * @return One of MPU_* error codes
 */
int8_t MPU9250::Enabled(bool en)
{
    HAL_MPU_PowerSwitch(en);

#ifdef __HAL_USE_EVENTLOG__
    if (en)
        EMIT_EV(-1, EVENT_UNINITIALIZED);
    else
        EMIT_EV(-1, EVENT_STARTUP);
#endif  /* __HAL_USE_EVENTLOG__ */

    return MPU_SUCCESS;
}

/**
 * Check if new sensor data has been received
 * @return true if new sensor data is available
 *        false otherwise
 */
bool MPU9250::IsDataReady()
{
    //  Call HAL to read the sensor interrupt pin
    return HAL_MPU_DataAvail();
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
 * Trigger reading data from MPU9250
 * Read data from MPU9250 and run AHRS algorithm when done
 * @return One of MPU_* error codes
 */
int8_t MPU9250::ReadSensorData()
{
    int16_t gyro[3], accel[3], mag[3];

    //  Read sensor data into buffers
    readAccelData(accel);
    readGyroData(gyro);
    //  Check if we're asked to read magnetometer
    if (_magEn)
        readMagData(mag);
    else
        memset((void*)mag, 0, 3*sizeof(uint16_t));

    //  Conversion from digital sensor readings to actual values
    for (uint8_t i = 0; i < 3; i++)
    {
        _acc[i] = (float)accel[i] * getAres();  //  m/s^2
        _gyro[i] = (float)gyro[i] * getGres();  //  deg/s
        _mag[i] = (float)mag[i] * getMres();    //  mG
    }

    //  Update attitude with new sensor readings
    _ahrs.Update(_gyro[0], _gyro[1], _gyro[2],
                 _acc[0], _acc[1], _acc[2],
                 _mag[0], _mag[1], _mag[2]);

    //  Copy data from AHRS object to this one
    memcpy((void*)_ypr, (void*)_ahrs.ypr, 3*sizeof(float));

    return MPU_SUCCESS;
}

/**
 * Copy orientation from internal buffer to user-provided one
 * @param RPY pointer to float buffer of size 3 to hold roll-pitch-yaw
 * @param inDeg if true RPY returned in degrees, if false in radians
 * @return One of MPU_* error codes
 */
int8_t MPU9250::RPY(float* RPY, bool inDeg)
{
    //  Copy data from internal buffer to a user-provided one, perform
    //  conversion from radians to degrees if asked
    for (uint8_t i = 0; i < 3; i++)
        if (inDeg)
            RPY[i] = _ypr[2-i]*180.0/PI_CONST;
        else
            RPY[i] = _ypr[2-i];

    return MPU_SUCCESS;
}

/**
 * Copy acceleration from internal buffer to user-provided one
 * @param acc Pointer a float array of min. size 3 to store 3-axis acceleration
 *        data
 * @return One of MPU_* error codes
 */
int8_t MPU9250::Acceleration(float *acc)
{
    //  Copy data from internal buffer to a user-provided one
    memcpy((void*)acc, (void*)_acc, sizeof(float)*3);

    return MPU_SUCCESS;
}

/**
 * Copy angular rotation from internal buffer to user-provided one
 * @param gyro Pointer a float array of min. size 3 to store 3-axis rotation
 *        data
 * @return One of MPU_* error codes
 */
int8_t MPU9250::Gyroscope(float *gyro)
{
    //  Copy data from internal buffer to a user-provided one
    memcpy((void*)gyro, (void*)_gyro, sizeof(float)*3);

    return MPU_SUCCESS;
}

/**
 * Copy mag. field strength from internal buffer to user-provided one
 * @param mag Pointer a float array of min. size 3 to store 3-axis mag. field
 *        strength data
 * @return One of MPU_* error codes
 */
int8_t MPU9250::Magnetometer(float *mag)
{
    //  Copy data from internal buffer to a user-provided one
    memcpy((void*)mag, (void*)_mag, sizeof(float)*3);

    return MPU_SUCCESS;
}

/**
 * Configure settings of AHRS algorithm
 * @note Using dT=0 will not update the value of dT in AHRS. This can be used
 * when one wants to update only the gains
 * @param dT Sampling time (time step between measurements)
 * @param kp Proportional gain
 * @param ki Integral gain
 * @return One of MPU_* error codes
 */
int8_t MPU9250::SetupAHRS(float dT, float kp, float ki)
{
    _ahrs.twoKi = 2* ki;
    _ahrs.twoKp = 2* kp;

    if (dT != 0.0f)
        _ahrs.InitSW(dT);

    return MPU_SUCCESS;
}

///-----------------------------------------------------------------------------
///                      Class constructor & destructor              [PROTECTED]
///-----------------------------------------------------------------------------

MPU9250::MPU9250() :  dT(0), _magEn(true), _ahrs()
{
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_UNINITIALIZED);
#endif  /* __HAL_USE_EVENTLOG__ */

    //  Initialize arrays
    memset((void*)_ypr, 0, 3);
    memset((void*)_acc, 0, 3);
    memset((void*)_gyro, 0, 3);
    memset((void*)_mag, 0, 3);
}

MPU9250::~MPU9250()
{}

#endif  /* __HAL_USE_MPU9250_NODMP__ */
