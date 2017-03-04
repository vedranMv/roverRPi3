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
 * Deprecated - used in single-file HAL hal_tm4c1294.h
 */
//#define _HAL_USE_ESP8266__
//#define _HAL_USE_ENGINES__
//#define _HAL_USE_RADAR__
//#define _HAL_USE_MPU9250__
//#define _HAL_USE_TASKSCH__


//  Define kernel modules for which is necessary to compile HAL interface
#define __HAL_USE_ESP8266__
#define __HAL_USE_ENGINES__
#define __HAL_USE_RADAR__
#define __HAL_USE_MPU9250__
#define __HAL_USE_TASKSCH__

//  Define sensor for sensor library
#define __MPU9250

#endif
