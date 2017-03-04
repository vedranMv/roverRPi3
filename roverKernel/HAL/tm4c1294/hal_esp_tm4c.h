/**
 * hal_esp_tm4c.h
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran Mikov
 *
 ****Hardware dependencies:
 *      UART7, pins PC4(Rx), PC5(Tx)
 *      GPIO PC6(CH_PD), PC7(Reset-not implemented!)
 *      Timer 6 - watchdog timer in case UART port hangs(likes to do so)
 */
#include "roverKernel/hwconfig.h"

//  Compile following section only if hwconfig.h says to include this module
#if !defined(ROVERKERNEL_HAL_TM4C1294_HAL_ESP_TM4C_H_) && defined(__HAL_USE_ESP8266__)
#define ROVERKERNEL_HAL_TM4C1294_HAL_ESP_TM4C_H_

#ifdef __cplusplus
extern "C"
{
#endif

extern uint32_t    HAL_ESP_InitPort(uint32_t baud);
extern void        HAL_ESP_RegisterIntHandler(void((*intHandler)(void)));
extern void        HAL_ESP_HWEnable(bool enable);
extern bool        HAL_ESP_IsHWEnabled();
extern void        HAL_ESP_IntEnable(bool enable);
extern int32_t     HAL_ESP_ClearInt();
extern bool        HAL_ESP_UARTBusy();
extern void        HAL_ESP_SendChar(char arg);
extern bool        HAL_ESP_CharAvail();
extern char        HAL_ESP_GetChar();
extern void        HAL_ESP_InitWD(void((*intHandler)(void)));
extern void        HAL_ESP_WDControl(bool enable, uint32_t timeout);
extern void        HAL_ESP_WDClearInt();

#ifdef __cplusplus
}
#endif


#endif /* ROVERKERNEL_HAL_TM4C1294_HAL_ESP_TM4C_H_ */
