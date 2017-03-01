/**
 * mpu9250.cpp
 *
 *  Created on: 25. 3. 2015.
 *      Author: Vedran
 */

#include "driverlib/fpu.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "roverKernel/tm4c1294_hal.h"
#include "mpu9250.h"
#include "myLib.h"

/*
 * NOTES:
 * 	?? X & Y axis are read from registers with opposite signs, as -X and -Y
 */

//  Number of samples to use when calibrating sensor (calculating offset from 0)
#define CALIBRATION_SAMPLES 200

//  Constants used for calculating/converting
const float GRAVITY_CONST = 9.80665f;	//	m/s^2

//  Pointer to the first created instance of MPU9250
MPU9250 *__mpu;

#if defined(__USE_TASK_SCHEDULER__)

/**
 * Callback routine to invoke service offered by this module from task scheduler
 * @note It is assumed that once this function is called task scheduler has
 * already copied required variables into the memory space provided for it.
 */
void _MPU_KernelCallback(void)
{
    //  Check for null-pointer
    if (__mpu->_mpuKer.argN == 0)
        return;
    /*
     *  Data in args[] contains bytes that constitute arguments for function
     *  calls. The exact representation(i.e. whether bytes represent ints, floats)
     *  of data is known only to individual blocks of switch() function. There
     *  is no predefined data separator between arguments inside args[].
     */
    switch (__mpu->_mpuKer.serviceID)
    {
    case MPU_LISTEN
        {

        }
        break;
    default:
        break;
    }
}
#endif

/*******************************************************************************
 *******************************************************************************
 *********              MPU9250 class member functions                 *********
 *******************************************************************************
 ******************************************************************************/

///-----------------------------------------------------------------------------
///                      Class constructor & destructor                [PUBLIC]
///-----------------------------------------------------------------------------

MPU9250::MPU9250() : dT(0), _dataFlag(false), userHook(0)
{
    memset((void*)(&_rawData[0][0]), 0, sizeof(_rawData));
    memset(_off, 0, sizeof(_off));
    memset(_range, 0, sizeof(_range));

    //  Ensure that only first instance of this class is saved in global pointer
    if (__mpu == 0)
        __mpu = this;
}

MPU9250::~MPU9250()
{
    __mpu = 0;
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
    HAL_MPU_Init(dataISR);

    return MPU_SUCCESS;
}

/**
 * Initialize MPU sensor:
 *      -Force it to use PLL for system clock (if fails uses 20MHz IOSC)
 *      -Sets I2C master frequency for MPU to 348kHz for talking to peripherals
 *      -Set sampling rate to 200Hz -> 1kHz sampling  /(divider=4(+1)) =>dt=0.005
 *      -Reduce accelerometer bandwidth to 5Hz TODO: play with settings
 *      -Configure interrupt pin to go high for 50uS on every interrupt event
 *          interrupt is cleared by reading any of the registers
 *      -Disable FSYNC feature
 *      -Data from accelerometer is written in reg. only if it's different from
 *          current value
 *      -Only change in raw data triggers interrupt
 */
int8_t MPU9250::InitSW()
{
    /*
     * Configure PLL and set sampling rate for accel. & gyro
     */
    // Reset MPU & wait 10ms for it to power up
    HAL_MPU_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
    HAL_DelayUS(10000);
    // Power-on, try PLL, if not use IOSC 20MHz
    HAL_MPU_WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    //  Set I2C freq to 400kHz
    HAL_MPU_WriteByte(MPU9250_ADDRESS, I2C_MST_CTRL , 0x00);
    //  Set sample divider to 4(+1)=5
    HAL_MPU_WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);
    //  Prepare for 200Hz bandwidth @ 1kHz/5
    HAL_MPU_WriteByte(MPU9250_ADDRESS, CONFIG, 0x01);
    //  Disable gyro self-test, set scale of 250dps,
    //  confirm 250Hz bandwidth@1kHz/5
    HAL_MPU_WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
    //  Accel is default +/-2g scale, self-test turned off
    //  Configure accel for 5Hz bandwidth @ 1kHz/5 sampling rate
    HAL_MPU_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x06);

    /*
     * Interrupt pin configuration
     */
    //  Allow accel to trigger interrupt on compare mismatch
    HAL_MPU_WriteByte(MPU9250_ADDRESS, MOT_DETECT_CTRL, 1<<6);
    //  Configure push-pull INT pin to go high when data is ready
    //  Clear interrupt by reading any register; enable bypass
    HAL_MPU_WriteByte(MPU9250_ADDRESS, INT_PIN_CFG,  (1<<4) | (1<<1));
    //  Activate interrupt when raw sensor data is available
    HAL_MPU_WriteByte(MPU9250_ADDRESS, INT_ENABLE, RAW_DATA_READY);

    /*
     * Adjust range of sensors
     */
    //  Set gyro range to 250dps
    SetRange(MPU_SENS_GYRO, RANGE_250DPS);
    //  Set accel range to +/-2g
    SetRange(MPU_SENS_ACC, RANGE_2G);

    //  If ID is wrong means there's an error in communication protocol
    if (GetID() == 0x71)
        return MPU_SUCCESS;
    else
        return MPU_I2C_ERROR;
}

/**
 * Trigger software reset of the MPU module
 */
void MPU9250::Reset()
{
    HAL_MPU_WriteByteNB(MPU9250_ADDRESS, PWR_MGMT_1, 1 << 7);
    HAL_DelayUS(50000);
}

/**
 * Updates range in register on MPU, and computes appropriate multiplier
 *  for converting 16bit ADC values from MPU to acceleration(m/s^2)
 *  or angular velocity (�/s)
 */
void MPU9250::SetRange(uint8_t sensor, uint8_t range)
{
    uint8_t temp;

    if (sensor ==  MPU_SENS_GYRO)
    {
        //  Read current range
        temp = HAL_MPU_ReadByte(MPU9250_ADDRESS, GYRO_CONFIG);
        //  Clear current range configuration
        temp = temp & ~(3<<3);
        //  Write new range configuration
        HAL_MPU_WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, temp | (range<<3));
        //  Update internal (in class object) value of range
        switch (range)
        {
        case RANGE_250DPS:
            _range[MPU_SENS_GYRO] = 250;
            break;
        case RANGE_500DPS:
            _range[MPU_SENS_GYRO] = 500;
            break;
        case RANGE_1000DPS:
            _range[MPU_SENS_GYRO] = 1000;
            break;
        case RANGE_2000DPS:
            _range[MPU_SENS_GYRO] = 2000;
            break;
        }
        //  Internal range is used to convert raw sensor value (signed 16 bit)
        //  to float
        _range[MPU_SENS_GYRO] = _range[MPU_SENS_GYRO]/32768.0f;
    }

    if (sensor == MPU_SENS_ACC)
    {
        //  Read current range
        temp = HAL_MPU_ReadByte(MPU9250_ADDRESS, ACCEL_CONFIG);
        //  Clear current range configuration
        temp = temp & ~(3<<3);
        //  Write new range configuration
        HAL_MPU_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, temp | (range<<3));
        //  Update internal (in class object) value of range
        switch (range)
        {
        case RANGE_2G:
            _range[MPU_SENS_ACC] = 2;
            break;
        case RANGE_4G:
            _range[MPU_SENS_ACC] = 4;
            break;
        case RANGE_8G:
            _range[MPU_SENS_ACC] = 8;
            break;
        case RANGE_16G:
            _range[MPU_SENS_ACC] = 16;
            break;
        }
        //  Internal range is used to convert raw sensor value (signed 16 bit)
        //  to float
        _range[MPU_SENS_ACC] = GRAVITY_CONST * _range[MPU_SENS_ACC]/32768.0f;   //  m/s^2
    }
}

/**
 * Calibration of two MPU instruments based on the input arguments
 *      -current offset values are overridden and upon completion replaced with
 *          new ones calculated by averaging a number of samples
 *      -SAMPLE_NUMBER macro defines number of samples to be averaged for calculation
 *      -function in principle does 'zeroing' of both instruments on all 3 axes
 *      -ASSUMPTION!!!:
 *          -gravity acts in direction of -X axis on the sensor
 *          -sensor is at rest, with only gravity acting on it
 */
int8_t MPU9250::Calibrate(bool accCal, bool gyroCal)
{
    uint8_t sampleCount = 0, i;
    float accSum[3] = {0,0,0},
          gyroSum[3]={0,0,0},
          aRes_old, gRes_old;

    //  If no calibration is required(but function still called)
    //  then return
    if (!(accCal | gyroCal)) return MPU_SUCCESS;

    dataISR();  //Call ISR to clear all interrupt flags
    //  Stop interrupts in order to prepare environment for calibration
    HAL_MPU_IntEnable(false);

    //  Prepare for calibrating accelerometer, if requested
    if (accCal)
    {
        //  Set resolution to 1 for easier calculation afterwards
        aRes_old = _range[MPU_SENS_ACC];
        _range[MPU_SENS_ACC] = 1;
        //  Clear old offset value
       _off[MPU_SENS_ACC][MPU_X_AXIS] = 0;
       _off[MPU_SENS_ACC][MPU_Y_AXIS] = 0;
       _off[MPU_SENS_ACC][MPU_Z_AXIS] = 0;
    }

    //  Prepare for calibrating gyro, if requested
    if (gyroCal)
    {
        //  Set resolution to 1 for easier calculation afterwards
        gRes_old = _range[MPU_SENS_GYRO];
        _range[MPU_SENS_GYRO] = 1;
        //  Clear old offset value
        _off[MPU_SENS_GYRO][MPU_X_AXIS] = 0;
        _off[MPU_SENS_GYRO][MPU_Y_AXIS] = 0;
        _off[MPU_SENS_GYRO][MPU_Z_AXIS] = 0;
    }

    //  Turn on interrupt-based data sampling and start acquiring samples
    HAL_MPU_IntEnable(true);
    while (sampleCount < CALIBRATION_SAMPLES)
    {
        //  Wait for the flag, then clear it for next cycle
        while (!IsDataReady());

        //  Add measurement from this cycle to common sum for each instrument and axis
        for (i = 0; i < 3; i++) accSum[i] += _rawData[MPU_SENS_ACC][i];
        for (i = 0; i < 3; i++) gyroSum[i] += _rawData[MPU_SENS_GYRO][i];

        _dataFlag = false;
        sampleCount++;
    }
    HAL_MPU_IntEnable(false);   //  Stop sampling so we can process data

    //  Calculate new offset based on readings
    //  offset=sumOfSamples/numberOfSamples
    if (accCal)
    {
        for (i = 0; i < 3; i++)
            _off[MPU_SENS_ACC][i] = (int16_t)(accSum[i]/((float)CALIBRATION_SAMPLES));

        //  Compensating for gravity acting fully on X-axis (-X direction)
        //  TODO: Detect direction of Earth's gravity
        _off[MPU_SENS_ACC][0] = _off[MPU_SENS_ACC][0] - GRAVITY_CONST / aRes_old;
        _range[MPU_SENS_ACC] = aRes_old;
    }

    if (gyroCal)
    {
        for (i = 0; i < 3; i++)
            _off[MPU_SENS_GYRO][i] = (int16_t)(gyroSum[i]/((float)CALIBRATION_SAMPLES));
        _range[MPU_SENS_GYRO] = gRes_old;
    }
    //  TODO: Write these values into MPU registers


    return MPU_SUCCESS;
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
}

/**
 * Read new data from all 3 instruments and save it member variables
 */
void MPU9250::ReadData()
{
    _GetRawAcc();
    _GetRawGyro();
    _GetRawMag();
    _dataFlag = true;
}

/**
 * Fetches raw sensor data from member variables and returns it in arrays
 * whose pointers are provided as arguments to this function
 * @param acc pointer to array to store accelerometer data (array size 3)
 * @param gyro pointer to array to store gyroscope data (array size 3)
 * @param mag pointer to array to store magnetometer data (array size 3)
 * @param clrFlag if true clears data-ready flag of this instance before it retuns
 */
void MPU9250::GetData(float* acc, float *gyro, float *mag, bool clrFlag)
{
    if (gyro != 0)
        for (int i = 0; i < 3; i++)
            gyro[i] = _rawData[MPU_SENS_GYRO][i];

    if (acc != 0)
        for (int i = 0; i < 3; i++)
            acc[i] = _rawData[MPU_SENS_ACC][i];

    if (mag != 0)
        for (int i = 0; i < 3; i++)
            mag[i] = _rawData[MPU_SENS_MAG][i];

    if (clrFlag) _dataFlag = false;
}

/**
 * Add hook to user-defined function to be called once new sensor
 * data has been received
 * @param custHook pointer to function with two float args (accel & gyro array)
 */
void MPU9250::AddHook(void((*custHook)(float*,float*)))
{
    userHook = custHook;
}

///-----------------------------------------------------------------------------
///         Public functions for reading raw data from MPU9250          [PUBLIC]
///-----------------------------------------------------------------------------

void MPU9250::_GetRawAcc()
{
    for (int i = MPU_X_AXIS; i <= MPU_Z_AXIS; i++)
    {
        uint8_t regData[2] = {0, 0};
        int16_t temp;

        // Read High & Low register for this particular axis
        HAL_MPU_ReadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H + 2 * i, 2, regData);
        temp = (int16_t)(((uint16_t)regData[0] << 8) | regData[1]);
        _rawData[MPU_SENS_ACC][i] = (float)(temp - _off[MPU_SENS_ACC][i]) * _range[MPU_SENS_ACC];
    }
}

void MPU9250::_GetRawGyro()
{
    for (int i = 0; i < 3; i++)
    {
        uint8_t regData[2] = {0, 0};
        int16_t temp;

        // Read High & Low register for this particular axis
        HAL_MPU_ReadBytes(MPU9250_ADDRESS, GYRO_XOUT_H + 2 * i, 2, regData);
        temp = (int16_t)(((uint16_t)regData[0] << 8) | regData[1]);
        _rawData[MPU_SENS_GYRO][i] = (float)(temp - _off[MPU_SENS_GYRO][i]) * _range[MPU_SENS_GYRO];
    }
}

void MPU9250::_GetRawMag()
{
    for (int i = 0; i < 3; i++)
    {
        uint8_t regData[2] = {0, 0};
        int16_t temp;

        // Read High & Low register for this particular axis
        HAL_MPU_ReadBytes(AK8963_ADDRESS, AK8963_XOUT_L + 2 * i, 2, regData);
        temp = (int16_t)(((uint16_t)regData[1] << 8) | regData[0]);
        _rawData[MPU_SENS_MAG][i] = (float)(temp - _off[MPU_SENS_MAG][i]) * _range[MPU_SENS_MAG];

    }
}

///-----------------------------------------------------------------------------
///         ISR executed whenever MPU toggles a data-ready pin          [EXTERN]
///-----------------------------------------------------------------------------


/**
 * On interrupt pin going high(PA2) this function gets called and does:
 *  1. clear TM4C interrupt status
 *  2. reads MPU interrupt status register to see what triggered
 *  3. if 'raw data available' flag is set, clear it and read raw sensor data
 *  Additionally: measures the time between interrupt calls to provide time
 *      reference for integration
 */
void dataISR(void)
{
    uint8_t intStatus;

    //  IntClear returns true if the interrupt was caused by the correct pin
    if (!HAL_MPU_IntClear()) return;

    //  Stop timer and get time reference from least measurement
    HAL_TIM_Stop();
    __mpu->dT = (float)HAL_TIM_GetValue()/(float)g_ui32SysClock;
    HAL_TIM_Start(0);

    //  Read MPU interrupt status to see what caused an interrupt
    intStatus = HAL_MPU_ReadByte(MPU9250_ADDRESS, INT_STATUS);

    if ((intStatus & RAW_DATA_READY) == RAW_DATA_READY)
    {
        float acc[3], gyro[3];

        __mpu->ReadData();
        __mpu->GetData(acc, gyro, 0, false);
        if (__mpu->userHook != 0)
            __mpu->userHook(acc, gyro);
        __mpu->IMU()->Update(acc, gyro);
    }
}

/**
 * Implementation of complementary filter to calculate R/P/Y
 * @param gyro
 * @param acc
 */
void Orientation::Update(float *acc, float *gyro)
{
    static bool firstEntry = true;
    float __R, __P, __Y,
          accR, accP;

    __R = gyro[MPU_Z_AXIS] * __mpu->dT;
    __P = gyro[MPU_Y_AXIS] * __mpu->dT;
    __Y = gyro[MPU_X_AXIS] * __mpu->dT;

    accR = -90 + atan2f(acc[MPU_X_AXIS], acc[MPU_Y_AXIS]) * 180 / PI_CONST;   //y,x
    accP =  90 - atan2f(acc[MPU_X_AXIS], acc[MPU_Z_AXIS]) * 180 / PI_CONST;

    if (firstEntry)
    {
        _RPYsetpoints[0] = __R;
        _RPYsetpoints[1] = __P;
        _RPYsetpoints[2] = __Y;
        firstEntry = false;
    }
    else
    {
        _roll = (__R + _roll) * 0.93 + (accR * 0.07);
        _pitch = (__P + _pitch) * 0.93 + (accP * 0.07);
        if (fabs(gyro[MPU_X_AXIS]) > 0.8) _yaw += __Y;

       /* _roll += __R;
        _pitch += __P;
        _yaw += __Y;*/

    }
}


