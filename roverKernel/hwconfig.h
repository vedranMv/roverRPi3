/**
 *  hwconfig.h
 *
 *  Created on: 03. 03. 2017.
 *      Author: Vedran Mikov
 *
 *  Hardware abstraction layer providing uniform interface between board support
 *  layer and hardware in there and any higher-level libraries. Acts as a
 *  switcher between HALs for different boards.
 */

#ifndef __HWCONFIG_H__
#define __HWCONFIG_H__

#include <stdint.h>
#include <stdbool.h>

//  Define platform in use in hal.h
#define __BOARD_TM4C1294NCPDT__

/*
 * Compile all libraries in debug mode, allowing them to print debug data to
 * serial port. This enabled debug session for ALL libraries. Individual
 * libraries can be debugged by setting this flag only in their headers.
 */
//#define __DEBUG_SESSION__

/*
 * Define kernel modules for which is necessary to compile HAL interface
 * In order to enable compilation of a module uncomment that module from
 * the following list.
 */
#define __HAL_USE_ESP8266__
#define __HAL_USE_ENGINES__

//#define __HAL_USE_MPU9250_NOSPI__
#define __HAL_USE_MPU9250_SPI__

#define __HAL_USE_RADAR__
#define __HAL_USE_TASKSCH__
#define __HAL_USE_EVENTLOG__

/*
 * This section configures MPU9250 sensor
 * MPU can either use SPI or I2C protocol
 * MPU can be configured to give raw data, or use DMP firmware
 */
#if defined(__HAL_USE_MPU9250_SPI__) || defined(__HAL_USE_MPU9250_NOSPI__)
    #define __HAL_USE_MPU9250__

    //  Use either one of these two!
    #define __HAL_USE_MPU9250_NODMP__
    //#define __HAL_USE_MPU9250_DMP__
#endif




//  Define number of modules in the kernel (used to initialize memory space)
#define NUM_OF_MODULES  10

//  Define sensor for sensor library
#define __MPU9250

#endif
