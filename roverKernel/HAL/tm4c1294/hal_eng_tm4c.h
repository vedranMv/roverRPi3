/*
 * hal_eng_tm4c.h
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran Mikov
 */
#include "roverKernel/hwconfig.h"

//  Compile following section only if hwconfig.h says to include this module
#if !defined(ROVERKERNEL_HAL_TM4C1294_HAL_ENG_TM4C_H_) && defined(__HAL_USE_ENGINES__)
#define ROVERKERNEL_HAL_TM4C1294_HAL_ENG_TM4C_H_

#ifdef __cplusplus
extern "C"
{
#endif

    extern void        HAL_ENG_Init(uint32_t pwmMin, uint32_t pwmMax);
    extern void        HAL_ENG_Enable(uint32_t engine, bool enable);
    extern uint8_t     HAL_ENG_SetPWM(uint32_t engine, uint32_t pwm);
    extern uint32_t    HAL_ENG_GetPWM(uint32_t engine);
    extern uint8_t     HAL_ENG_SetHBridge(uint32_t mask, uint8_t dir);
    extern uint32_t    HAL_ENG_GetHBridge(uint32_t mask);
    extern void        HAL_ENG_IntClear(uint32_t engine);
    extern void        HAL_ENG_IntEnable(uint32_t engine, bool enable);

#ifdef __cplusplus
}
#endif

#endif /* ROVERKERNEL_HAL_TM4C1294_HAL_ENG_TM4C_H_ */
