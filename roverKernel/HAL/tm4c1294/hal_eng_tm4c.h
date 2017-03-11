/**
 * hal_eng_tm4c.h
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran Mikov
 *
 ****Hardware dependencies:
 *      PWM0 - Generator 1
 *      PWM Out2(PF2 - left wheel), PWM Out3(PF3 - right wheel)
 *      GPIO PL0&PL1(left wheel), PL2&PL3(right wheel)
 *      GPIO PP0(left optical encoder), PP1(right optical encoder) - interrupt
 *      Interrupts have to be registered through startup_ccs.c file
 */
#include "roverKernel/hwconfig.h"

//  Compile following section only if hwconfig.h says to include this module
#if !defined(ROVERKERNEL_HAL_TM4C1294_HAL_ENG_TM4C_H_) && defined(__HAL_USE_ENGINES__)
#define ROVERKERNEL_HAL_TM4C1294_HAL_ENG_TM4C_H_

/**     Engines error codes                 */
#define HAL_ENG_PWMOOR          4   /// PWM value out of range
#define HAL_ENG_EOOR            5   /// Engine ID out of range
#define HAL_ENG_ILLM            6   /// Illegal mask for H-bridge configuration

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
