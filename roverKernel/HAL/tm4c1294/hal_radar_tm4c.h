/**
 * hal_radar_tm4c.h
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran Mikov
 *
 **** Hardware dependencies:
 *      PWM - Generator 1 & 3
 *      G1:PWM Out1(PF1 - horiz. axis), G3:PWM Out4(PG0 - vert. axis)
 *      ADC0: AIN3(PE0) - sampling sensor output
 */
#include "roverKernel/hwconfig.h"

//  Compile following section only if hwconfig.h says to include this module
#if !defined(ROVERKERNEL_HAL_TM4C1294_HAL_RADAR_TM4C_H_) && defined(__HAL_USE_RADAR__)
#define ROVERKERNEL_HAL_TM4C1294_HAL_RADAR_TM4C_H_

#ifdef __cplusplus
extern "C"
{
#endif

extern void        HAL_RAD_SetVerAngle(float angle);
extern float       HAL_RAD_GetVerAngle();
extern void        HAL_RAD_SetHorAngle(float angle);
extern float       HAL_RAD_GetHorAngle();
extern void        HAL_RAD_Init();
extern void        HAL_RAD_Enable(bool enable);
extern uint32_t    HAL_RAD_ADCTrigger();

#ifdef __cplusplus
}
#endif

#endif /* ROVERKERNEL_HAL_TM4C1294_HAL_RADAR_TM4C_H_ */
