/**
 * hal_ts_tm4c.c
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran
 */
#include "hal_ts_tm4c.h"

#if  defined(__HAL_USE_TASKSCH__)   //  Compile only if module is enabled

#include "libs/myLib.h"
#include "HAL/tm4c1294/hal_common_tm4c.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_ints.h"

#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"


uint32_t g_ui32SysClock;

/**
 * Setup SysTick interrupt and period
 * @param periodMs time in milliseconds how often to trigger an interrupt
 * @param custHook pointer to function that will be called on SysTick interrupt
 * @return HAL library error code
 */
///Keep track whether the SysTick has already been configured
bool _systickSet = false;
uint32_t _periodMS = 0;
uint8_t HAL_TS_InitSysTick(uint32_t periodMs,void((*custHook)(void)))
{
    /// Forbid configuring the timer period multiple times
    if (_systickSet)
        return HAL_SYSTICK_SET_ERR;
    /// Based on SysTick API 16777216 is biggest allowed number for SysTick
    if (16777216 < (periodMs*(g_ui32SysClock/1000)))
        return HAL_SYSTICK_PEROOR;
    /// And 1 minimum allowed
    if (1 > (periodMs*(g_ui32SysClock/1000)))
            return HAL_SYSTICK_PEROOR;

    MAP_SysTickPeriodSet(periodMs*(g_ui32SysClock/1000));
    SysTickIntRegister(custHook);
    MAP_IntPrioritySet(FAULT_SYSTICK, 0);
    MAP_SysTickIntEnable();
    _systickSet = true;

    _periodMS = periodMs;

    return 0;
}

/**
 * Wrapper for SysTick start function
 */
uint8_t HAL_TS_StartSysTick()
{
    if(_systickSet)
        MAP_SysTickEnable();
    else
        return HAL_SYSTICK_NOTSET_ERR;

    return 0;
}

/**
 * Wrapper for SysTick stop function
 */
uint8_t HAL_TS_StopSysTick()
{
    if(_systickSet)
        MAP_SysTickDisable();
    else
        return HAL_SYSTICK_NOTSET_ERR;

    return 0;
}

/**
 * Calculate time step between two SysTick interrupts (in milliseconds)
 * @return time step between two SysTicks (in ms)
 */
uint32_t HAL_TS_GetTimeStepMS()
{
    return _periodMS;
}

#endif  /* __HAL_USE_TASKSCH__ */

