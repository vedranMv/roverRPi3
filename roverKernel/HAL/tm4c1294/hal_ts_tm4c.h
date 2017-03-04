/**
 * hal_ts_tm4c.h
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran Mikov
 */
#include "roverKernel/hwconfig.h"

//  Compile following section only if hwconfig.h says to include this module
#if !defined(ROVERKERNEL_HAL_TM4C1294_HAL_TS_TM4C_H_) && defined(__HAL_USE_TASKSCH__)
#define ROVERKERNEL_HAL_TM4C1294_HAL_TS_TM4C_H_

#ifdef __cplusplus
extern "C"
{
#endif
/**     TaskScheduler - related API     */
extern uint8_t     HAL_TS_InitSysTick(uint32_t periodMs, void((*custHook)(void)));
extern uint8_t     HAL_TS_StartSysTick();
extern uint8_t     HAL_TS_StopSysTick();
extern uint32_t    HAL_TS_GetTimeStepMS();

/**     Test probes     */
extern void        HAL_ESP_TestProbe();

#ifdef __cplusplus
}
#endif

#endif /* ROVERKERNEL_HAL_TM4C1294_HAL_TS_TM4C_H_ */
