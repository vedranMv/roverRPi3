/**
 * mpu9250.cpp
 *
 *  Created on: 25. 3. 2015.
 *      Author: Vedran
 */
#include "mpu9250.h"

#if defined(__HAL_USE_MPU9250__)       //  Compile only if module is enabled

#include "HAL/hal.h"
#include "libs/myLib.h"
#include "libs/helper_3dmath.h"

#include "eMPL/inv_mpu.h"
#include "eMPL/inv_mpu_dmp_motion_driver.h"

//  Enable debug information printed on serial port
//#define __DEBUG_SESSION__

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

//  Function prototype to an interrupt handler (declared at the bottom)
void MPUDataHandler(void);
static volatile bool dF = false;

///-----------------------------------------------------------------------------
///         DMP related structs --  Start
///-----------------------------------------------------------------------------

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
//  In this case chip is rotated -90° around Y axis
static signed char gyro_orientation[9] = { 0, 0, -1,
                                           0, 1,  0,
                                           1, 0,  0};

struct int_param_s int_param;

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};
static struct hal_s hal = {0};

///-----------------------------------------------------------------------------
///         DMP related structs --  End
///-----------------------------------------------------------------------------

///-----------------------------------------------------------------------------
///         DMP related functions --  Start
///-----------------------------------------------------------------------------

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

///-----------------------------------------------------------------------------
///         DMP related functions --  End
///-----------------------------------------------------------------------------

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
     * retVal one of MPU_* error codes
     */
    case MPU_T_LISTEN:
        {
            //  Double negation to convert any non-zero int to bool
            bool listen = !(!(__mpu._mpuKer.args[0]));

            __mpu.Listen(listen);
            __mpu._mpuKer.retVal = MPU_SUCCESS;
        }
        break;

    /*
     *  Once new data is available, read it from FIFO and store in data structure
     */
    case MPU_T_GET_DATA:
        {
            if (__mpu._dataFlag)
            {
                short gyro[3], accel[3], sensors;
                unsigned char more;
                long quat[4];
                unsigned long sensor_timestamp;

            #ifdef __USE_TASK_SCHEDULER__
                //  Calculate dT in seconds!
                static uint64_t oldms = 0;
                __mpu.dT = (float)(msSinceStartup-oldms)/1000.0f;
                oldms = (int32_t)msSinceStartup;
            #endif /* __HAL_USE_TASKSCH__ */

                /* This function gets new data from the FIFO when the DMP is in
                 * use. The FIFO can contain any combination of gyro, accel,
                 * quaternion, and gesture data. The sensors parameter tells the
                 * caller which data fields were actually populated with new data.
                 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
                 * the FIFO isn't being filled with accel data.
                 * The driver parses the gesture data to determine if a gesture
                 * event has occurred; on an event, the application will be notified
                 * via a callback (assuming that a callback function was properly
                 * registered). The more parameter is non-zero if there are
                 * leftover packets in the FIFO.
                 */
                if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more) != 0)
                {
            #ifdef __HAL_USE_EVENTLOG__
                    EMIT_EV(MPU_T_GET_DATA, EVENT_HANG);
            #endif  /* __HAL_USE_EVENTLOG__ */
                    return;
                }

                Quaternion qt;
                qt.x = (float)quat[0]/QUAT_SENS;
                qt.y = (float)quat[1]/QUAT_SENS;
                qt.z = (float)quat[2]/QUAT_SENS;
                qt.w = (float)quat[3]/QUAT_SENS;

                VectorFloat v;
                dmp_GetGravity(&v, &qt);

                dmp_GetYawPitchRoll((float*)(__mpu._ypr), &qt, &v);

                __mpu._quat[0] = qt.x;
                __mpu._quat[1] = qt.y;
                __mpu._quat[2] = qt.z;
                __mpu._quat[3] = qt.w;

                //  Copy to MPU class
                __mpu._gv[0] = v.x;
                __mpu._gv[1] = v.y;
                __mpu._gv[2] = v.z;

                //  Do data health-check -> too big change in angle(30° cumulative)
                //  between consecutive readings points to error
                if ((fabs(sumOfRot - fabs(__mpu._ypr[0]) - fabs(__mpu._ypr[1]) -
                          fabs(__mpu._ypr[2])) > 0.5) && (sumOfRot != 0.0))
                {
                    //  Emit error event to the system if consecutive sensor
                    //   readings are too far off
#ifdef __HAL_USE_EVENTLOG__
                    EMIT_EV(MPU_T_GET_DATA, EVENT_ERROR);
#endif  /* __HAL_USE_EVENTLOG__ */
                    __mpu._mpuKer.retVal = MPU_ERROR;
                }
                else
                    __mpu._mpuKer.retVal = MPU_SUCCESS;

                //  Clear flag
                //dF = false;
                __mpu._dataFlag = false;

                sumOfRot = fabs(__mpu._ypr[0]) + fabs(__mpu._ypr[1]) + fabs(__mpu._ypr[2]);
            }

        }
        break;
        /*
         * Restart MPU module and reload DMP firmware
         */
    case MPU_T_REBOOT:
        {
            if (__mpu._mpuKer.args[0] == 0x17)
            {
                sumOfRot = 0.0; //Prevents error for big change in value after reboot
                __mpu.Reset();
                __mpu._mpuKer.retVal = (int32_t)__mpu.InitSW();
            }
        }
        break;
        /*
         * Soft reboot of MPU -> only reset status in event logger
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
 * frequency 1MHz, connection timeout: 100ms. Initializes interrupt pin(PA5)
 * to be toggled by MPU9250 when it has data available for reading (PA5 is
 * push-pull pin with weak pull down and 10mA strength). Initializes Timer 7 to
 * count time between 2 sensor measurements, to measure dT.
 */
int8_t MPU9250::InitHW()
{
    HAL_MPU_Init(MPUDataHandler);

    return MPU_SUCCESS;
}

/**
 * Initialize MPU sensor:
 */
int8_t MPU9250::InitSW()
{
    int result;

    //  Emit event before initializing module
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_STARTUP);
#endif  /* __HAL_USE_EVENTLOG__ */

#if defined(__USE_TASK_SCHEDULER__)
    //  Register module services with task scheduler
    _mpuKer.callBackFunc = _MPU_KernelCallback;
    TS_RegCallback(&_mpuKer, MPU_UID);
#endif

    mpu_init(&int_param);

    //  Get/set hardware configuration. Start gyro.
    // Wake up all sensors.
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    // Push accel and quaternion data into the FIFO.
    mpu_configure_fifo(INV_XYZ_ACCEL);
    mpu_set_sample_rate(10);//50

    // Initialize HAL state variables.
    memset(&hal, 0, sizeof(hal));
#ifdef __DEBUG_SESSION__
    DEBUG_WRITE("Trying to load firmware\n");
#endif
    result = 7; //  Try loading firmware max 7 times
    while(result--)
        if (dmp_load_motion_driver_firmware() == 0)
            break;
#ifdef __DEBUG_SESSION__
        else
            DEBUG_WRITE("%d,  ", result);
#endif

    if (result <= 0)    //  If loading failed 7 times hang here, DMP not usable
    {
#ifdef __DEBUG_SESSION__
        DEBUG_WRITE("   >failed\n");
#endif
        //  Emit error event if using event log, instead of hanging
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_ERROR);
    return MPU_ERROR;
#else
    while(1);
#endif  /* __HAL_USE_EVENTLOG__ */
    }

#ifdef __DEBUG_SESSION__
    DEBUG_WRITE(" >Firmware loaded\n");
    DEBUG_WRITE(" >Updating DMP features...");
#endif
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));

    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);

    dmp_set_fifo_rate(20);//50
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
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
 * Trigger software reset of the MPU module
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
 */
uint8_t MPU9250::GetID()
{
    uint8_t ID;
    ID = HAL_MPU_ReadByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

    return ID;
}

/**
 * Listen for new sensor measurements by enabling/disabling sensor interrupt pin
 * @param enable new status of sensor interrupt
 */
void MPU9250::Listen(bool enable)
{
    HAL_MPU_IntEnable(enable);
    _listen = enable;
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

///-----------------------------------------------------------------------------
///                      Class constructor & destructor              [PROTECTED]
///-----------------------------------------------------------------------------

MPU9250::MPU9250() : _listen(0), dT(0), _dataFlag(false), userHook(0)
{
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_UNINITIALIZED);
#endif  /* __HAL_USE_EVENTLOG__ */
}

MPU9250::~MPU9250()
{}


///-----------------------------------------------------------------------------
///         ISR executed whenever MPU toggles a data-ready pin         [PRIVATE]
///-----------------------------------------------------------------------------

/**
 * On interrupt pin going high(PA2) this function gets called and does:
 *  1. Clear TM4C interrupt status
 *  2. Set data-ready flag
 *  3. Schedule reading data from FIFO through task scheduler
 */
void MPUDataHandler(void)
{
    MPU9250 &__mpu = MPU9250::GetI();

    HAL_MPU_IntClear();

    //  Raise new-data flag
    __mpu._dataFlag = true;
}

#endif  /* __HAL_USE_MPU9250__ */
