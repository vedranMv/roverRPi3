/**
 * MPU9250.h
 *
 *  Created on: 25. 3. 2015.
 *      Author: Vedran Mikov
 *  @version V1.2
 *  V1.0 - 25.3.2016
 *  +MPU9250 library now implemented as a C++ object
 *  V1.1 - 25.6.2016
 *  +New class Orientation added in order to provide single interface for position data
 *  V1.2 - 25.2.2017
 *  +Integration with task scheduler
 *  V1.2.1 - 11.3.2017
 *  +Changed MPU9250 class into a singleton
 *  V3.0 - 29.5.2017
 *  +Completely rewriting MPU9250 class, uses built-in digital motion processor
 *  instead of reading raw sensor data.
 */
#include "roverKernel/hwconfig.h"

//  Compile following section only if hwconfig.h says to include this module
#if !defined(ROVERKERNEL_MPU9250_MPU9250_H_) && defined(__HAL_USE_MPU9250__)
#define ROVERKERNEL_MPU9250_MPU9250_H_

#include "registerMap.h"

//  Enable debug information printed on serial port
#define __DEBUG_SESSION__

//  Enable integration of this library with task scheduler but only if task
//  scheduler is being compiled into this project
#if defined(__HAL_USE_TASKSCH__)
#define __USE_TASK_SCHEDULER__
#endif  /* __HAL_USE_TASKSCH__ */

//  Check if this library is set to use task scheduler
#if defined(__USE_TASK_SCHEDULER__)
    #include "roverKernel/taskScheduler/taskScheduler.h"
    //  Unique identifier of this module as registered in task scheduler
    #define MPU_UID             3
    //  Definitions of ServiceID for service offered by this module
    #define MPU_LISTEN          0
    #define MPU_GET_DATA        1
#endif

//Custom error codes for the library
#define MPU_SUCCESS             0
#define MPU_I2C_ERROR           1


/**
 * Class object for MPU9250 sensor
 */
class MPU9250
{
    friend void _MPU_KernelCallback(void);
    friend void MPUDataHandler(void);
    public:
        static MPU9250& GetI();
        static MPU9250* GetP();

        int8_t  InitHW();
        int8_t  InitSW();
        void    Reset();
        bool    IsDataReady();
        uint8_t GetID();

        void    Listen(bool enable);

        void    AddHook(void((*custHook)(uint8_t,float*)));

        volatile float  dT;
        //  Function to be hooked when new sensor data is received. 1st argument
        //  is what kind of data to pass, 2nd is float array to store data
        void((*userHook)(uint8_t,float*));
    protected:
        MPU9250();
        ~MPU9250();
        MPU9250(MPU9250 &arg) {}              //  No definition - forbid this
        void operator=(MPU9250 const &arg) {} //  No definition - forbid this

        //  Orientation in quaternions in sensor units
        volatile float _quat[4];
        //  Yaw-Pitch-Roll orientation[Y,P,R] in radians
        volatile float _ypr[3];
        //  Gravity vector[x,y,z]
        volatile float _gv[3];

        //  Flag set by ISR whenever new raw data is available
        volatile bool   _dataFlag;

#if defined(__USE_TASK_SCHEDULER__)
        _kernelEntry _mpuKer;
#endif
};

#endif /* MPU9250_H_ */
