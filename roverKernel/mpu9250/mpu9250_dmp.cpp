/**
 * mpu9250_dmp.cpp
 *
 *  Created on: 1. 3. 2017.
 *      Author: Vedran
 *
 *  @note Code is adaptation of MotionApps4.1 library from:
 *      https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU9150
 */

#include "../HAL/tm4c1294_hal.h"
#include "mpu9250.h"
#include "../libs/myLib.h"

#define MPU9150_DMP_CODE_SIZE       1962    // dmpMemory[]
#define MPU9150_DMP_CONFIG_SIZE     232     // dmpConfig[]
#define MPU9150_DMP_UPDATES_SIZE    140     // dmpUpdates[]
#define MPU9150_DMP_MEMORY_BANKS        8
#define MPU9150_DMP_MEMORY_BANK_SIZE    256
#define MPU9150_DMP_MEMORY_CHUNK_SIZE   16

