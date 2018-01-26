/**
 * mpu9250.cpp
 *
 *  Created on: 25. 3. 2015.
 *      Author: Vedran
 */
#include "mpu9250.h"

#if defined(__HAL_USE_MPU9250_NODMP__)       //  Compile only if module is enabled

#include "HAL/hal.h"
#include "libs/myLib.h"
#include "libs/helper_3dmath.h"


//  Enable debug information printed on serial port
#define __DEBUG_SESSION__

//  Sensitivity of sensor while outputting quaternions
#define QUAT_SENS  1073741824.0f

//  Integration with event log, if it's present
#ifdef __HAL_USE_EVENTLOG__
    #include "init/eventLog.h"
    //  Simplify emitting events
    #define EMIT_EV(X, Y)  EventLog::EmitEvent(MPU_UID, X, Y)
#endif  /* __HAL_USE_EVENTLOG__ */

#ifdef __DEBUG_SESSION__
#include "serialPort/uartHW.h"
#endif

static uint8_t gScale = GFS_250DPS;
static uint8_t aScale = AFS_2G;
static uint8_t mScale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution

static float accelScale = 1.0f;
static float gyroScale = 1.0f;


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

            if (HAL_MPU_DataAvail())
            {
                int8_t retVal;
                short gyro[3], accel[3], sensors;
                unsigned char more = 1;
                long quat[4];
                unsigned long sensor_timestamp;

            #ifdef __USE_TASK_SCHEDULER__
                //  Calculate dT in seconds!
                static uint64_t oldms = 0;
                __mpu.dT = (float)(msSinceStartup-oldms)/1000.0f;
                oldms = msSinceStartup;
            #endif /* __HAL_USE_TASKSCH__ */

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
    int result;

    //  Power cycle MPU chip on every SW initialization
    HAL_MPU_PowerSwitch(false);
    HAL_DelayUS(20000);
    HAL_MPU_PowerSwitch(true);
    HAL_DelayUS(30000);

    //  Emit event before initializing module
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_STARTUP);
#endif  /* __HAL_USE_EVENTLOG__ */

    HAL_MPU_WriteByte(MPU9250_ADDRESS, ADDR_PWR_MGMT_1, CLOCK_SEL_PLL);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ADDR_USER_CTRL, I2C_MST_EN);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ADDR_I2C_MST_CTRL, I2C_MST_CLK);
        writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ADDR_PWR_MGMT_1, PWR_RESET);
        writeAK8963Register(AK8963_CNTL2, AK8963_RESET);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ADDR_PWR_MGMT_1, CLOCK_SEL_PLL);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ADDR_PWR_MGMT_2, SEN_ENABLE);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ADDR_ACCEL_CONFIG, AFS_4G);
    if(HAL_MPU_ReadByte(MPU9250_ADDRESS, ADDR_ACCEL_CONFIG) != AFS_4G)
    {
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_ERROR);
#endif  /* __HAL_USE_EVENTLOG__ */
        while(1);
    }
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ADDR_ACCEL_CONFIG, aScale);
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ADDR_GYRO_CONFIG, gScale);

    // Continue here, AK setup
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ADDR_ACCEL_CONFIG2, ACCEL_DLPF_41); //41
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ADDR_CONFIG, GYRO_DLPF_41); // 41
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ADDR_SMPLRT_DIV, 2);
    uint8_t smplrtDivCheck = HAL_MPU_ReadByte(MPU9250_ADDRESS, ADDR_SMPLRT_DIV);
    if(smplrtDivCheck != 2)
    {
#ifdef __HAL_USE_EVENTLOG__
EMIT_EV(-1, EVENT_ERROR);
#endif  /* __HAL_USE_EVENTLOG__ */
    }
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ADDR_INT_PIN_CFG, 1<<5);    //Held high until cleared
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ADDR_INT_ENABLE, INT_RAW_RDY_EN);

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
    HAL_MPU_WriteByteNB(MPU9250_ADDRESS, ADDR_PWR_MGMT_1, 1 << 7);
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
    ID = HAL_MPU_ReadByte(MPU9250_ADDRESS, ADDR_WHO_AM_I);

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
