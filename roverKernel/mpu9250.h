/**
 * mpu9250.h
 *
 *  Created on: 25. 3. 2015.
 *      Author: Vedran
 *  @version V1.2
 *  V1.0 - 25.3.2016
 *  +MPU9250 library now implemented as a C++ object
 *  V1.1 - 25.6.2016
 *  +New class Orientation added in order to provide single interface for position data
 *  V1.2 - 25.2.2017
 *  +Integration
 ****Hardware dependencies:
 *  Timer 7 - measuring dT, time step used in integration
 *  GPIO A5 - Data Ready interrupt pin
 *  I2C 2   - Data transportation
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include "tm4c1294_hal.h"

//  Enable integration of this library with task scheduler
#define __USE_TASK_SCHEDULER__

#if defined(__USE_TASK_SCHEDULER__)
    #include "taskScheduler.h"
    //  Unique identifier of this module as registered in task scheduler
    #define MPU_UID             3
    //  Definitions of ServiceID for service offered by this module
    #define MPU_LISTEN          0
    #define MPU_GET_DATA        1
#endif

/*      Device addresses        */
#define AK8963_ADDRESS   0x0C // Magnetometer I2C address
#define MPU9250_ADDRESS  0x68 // Device address when ADO = 0

/*      Magnetometer Register map   */
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

/*      MPU9250 Register map        */
#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02
//  Registers 3-12 not available
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B  // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

/*
 * Range configuration parameters
 * Gyro: 1 (+/-250�/s), 2 (+/-500�/s), 3 (+/-1000�/s), 4 (+/-2000�/s),
 * Accel: 1 (+/-2g), 2 (+/-4g), 3 (+/-8g), 4 (+/-16g),
 * Mag:
 */
#define RANGE_2G 		0
#define RANGE_4G 		1<<3
#define RANGE_8G 		2<<3
#define RANGE_16G 		3<<3

#define RANGE_250DPS 	0
#define RANGE_500DPS	1<<3
#define RANGE_1000DPS	2<<3
#define RANGE_2000DPS	3<<3

/*
 * REG_INT_ENABLE configurations
 * 	-enable interrupt on following events
 * REG_INT_FLAGS
 * 	-use same bit to check for occurred interrupts
 */
#define WAKE_ON_MOTION	1<<6
#define FIFO_OVERFLOW	1<<4
#define RAW_DATA_READY	1

//Custom error codes for the library
#define MPU_SUCCESS 			0
#define MPU_I2C_ERROR 			1


#define MPU_SENS_GYRO   0
#define MPU_SENS_ACC    1
#define MPU_SENS_MAG    2

#define MPU_X_AXIS      0
#define MPU_Y_AXIS      1
#define MPU_Z_AXIS      2

#define MPU_THRESH      15.0f

class Orientation;
class MPU9250;

/**
 * Object containing orientation data calculated from raw sensor readings
 */
class Orientation
{
    friend class MPU9250;
    friend void dataISR(void);
    public:
        Orientation(): _roll(0),_pitch(0), _yaw(0)
        {
            _RPYsetpoints[0]=_RPYsetpoints[1]=_RPYsetpoints[2] = 0;
        }
        void Update(float *acc, float *gyro);
        void GetOrientation(float *roll, float *pitch, float *yaw)
        {
            *roll = _roll;
            *pitch = _pitch;
            *yaw = _yaw;
        }

    protected:
        //  RPY orientation
        float _roll;
        float _pitch;
        float _yaw;
        //  Set-point for RPY
        float _RPYsetpoints[3];
        //  Direction of gravity
        float _gravityVect;
};

/**
 * Class object for MPU9250 sensor
 */
class MPU9250
{
    friend void _MPU_KernelCallback(void);
    public:
        MPU9250();
        ~MPU9250();

        int8_t  InitHW();
        int8_t  InitSW();
        void    Reset();
        void    SetRange(uint8_t sensor, uint8_t range);
        int8_t  Calibrate(bool accCal, bool gyroCal);
        bool    IsDataReady();
        uint8_t GetID();

        void    Listen(bool enable);
        void    ReadData();
        void    GetData(float* gyro, float *acc, float *mag, bool clrFlag = true);

        void    AddHook(void((*custHook)(float*,float*)));

        Orientation*    IMU() { return &_ort; }

        volatile float  dT;
        //  Function to be hooked when new sensor data is received. 1st argument
        //  is float[3] array for acceleration, 2nd is float[3] for gyroscope
        void((*userHook)(float*,float*));
    protected:
        void    _GetRawGyro();
        void    _GetRawAcc();
        void    _GetRawMag();

        //  Raw sensor data [0][]-3axis gyro, [1][]-3axis accel, [2][]-3axis mag
        //  {{x,y,z}�/s, {x,y,z}m/s^2, {x,y,z}}
        volatile float  _rawData[3][3];
        //  Flag set by ISR whenever new raw data is available
        volatile bool   _dataFlag;
        //  Holds measuring range of all 3 instruments: +/-_range[]
        float           _range[3];
        //  Offset of instruments at all 3 axes
        float           _off[3][3];
        //  Holds orientation calculated from raw sensor data
        Orientation     _ort;
#if defined(__USE_TASK_SCHEDULER__)
        _kernelEntry _mpuKer;
#endif
};


extern void dataISR(void);

#endif /* MPU9250_H_ */
