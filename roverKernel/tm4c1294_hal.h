/**
 *	tm4c1294_hal.h
 *
 *  Created on: 07. 08. 2016.
 *      Author: Vedran Mikov
 *
 *  Hardware abstraction layer for TM4C1294 microcontroller. It's not directly
 *  tied to hardware as it doesn't work with registers of MCU but rather uses
 *  a TivaWare library provided by TI.
 *
 **************Hardware dependencies
 *  ESP8266:
 *      UART7, pins PC4(Rx), PC5(Tx)
 *      GPIO PC6(CH_PD), PC7(Reset-not implemented!)
 *  Engines:
 *      PWM0 - Generator 1
 *      PWM Out2(PF2 - left wheel), PWM Out3(PF3 - right wheel)
 *      GPIO PL0&PL1(left wheel), PL2&PL3(right wheel)
 *      GPIO PP0(left optical encoder), PP1(right optical encoder) - interrupt
 *  Radar:
 *      PWM - Generator 1 & 3
 *      G1:PWM Out1(PF1 - horiz. axis), G3:PWM Out4(PG0 - vert. axis)
 *      ADC0: AIN3(PE0) - sampling sensor output
 *  MPU9250:
 *      I2C2 - Communication with sensor, pins PN4(SDA) & PN5(SCL)
 *      GPIO PA5 - Data available interrupt
 *      Timer 7 - Measure time between two consecutive sensor measurements
 *  Task scheduler:
 *      SysTick timer - interrupt based
 */
#ifndef TM4C1294_HAL_H_
#define TM4C1294_HAL_H_


#define HAL_OK                  0
/**     SysTick peripheral error codes      */
#define HAL_SYSTICK_PEROOR      1   /// Period value for SysTick is out of range
#define HAL_SYSTICK_SET_ERR     2   /// SysTick has already been configured
#define HAL_SYSTICK_NOTSET_ERR  3   /// SysTick hasn't been configured yet
/**     Engines error codes                 */
#define HAL_ENG_PWMOOR          4   /// PWM value out of range
#define HAL_ENG_EOOR            5   /// Engine ID out of range
#define HAL_ENG_ILLM            6   /// Illegal mask for H-bridge configuration

#ifdef __cplusplus
extern "C"
{
#endif


/// Global clock variable
extern uint32_t g_ui32SysClock;

extern void HAL_DelayUS(uint32_t us);
extern void HAL_BOARD_CLOCK_Init();


/**     ESP8266 - related HW API      */
    extern uint32_t    HAL_ESP_InitPort(uint32_t baud);
    extern void        HAL_ESP_RegisterIntHandler(void((*intHandler)(void)));
    extern void        HAL_ESP_HWEnable(bool enable);
    extern bool        HAL_ESP_IsHWEnabled();
    extern void        HAL_ESP_IntEnable(bool enable);
    extern void        HAL_ESP_ClearInt();
    extern bool        HAL_ESP_UARTBusy();
    extern void        HAL_ESP_SendChar(char arg);
    extern bool        HAL_ESP_CharAvail();
    extern char        HAL_ESP_GetChar();
    extern void        HAL_ESP_InitWD(void((*intHandler)(void)));
    extern void        HAL_ESP_WDControl(bool enable);
    extern void        HAL_ESP_WDClearInt();
/**     Engines - related HW API      */
    extern void        HAL_ENG_Init(uint32_t pwmMin, uint32_t pwmMax);
    extern void        HAL_ENG_Enable(bool enable);
    extern uint8_t     HAL_ENG_SetPWM(uint32_t engine, uint32_t pwm);
    extern uint8_t     HAL_ENG_SetHBridge(uint32_t mask, uint8_t dir);
    extern uint32_t    HAL_ENG_GetHBridge(uint32_t mask);
    extern void        HAL_ENG_IntClear(uint32_t engine);
    extern void        HAL_ENG_IntEnable(uint32_t engine, bool enable);
/**     Radar - related HW API        */
    extern void        HAL_RAD_SetVerAngle(float angle);
    extern float       HAL_RAD_GetVerAngle();
    extern void        HAL_RAD_SetHorAngle(float angle);
    extern float       HAL_RAD_GetHorAngle();
    extern void        HAL_RAD_Init();
    extern void        HAL_RAD_Enable(bool enable);
    extern uint32_t    HAL_RAD_ADCTrigger();
/**     MPU9250(I2C) - related HW API       */
    extern void        HAL_MPU_Init(void((*custHook)(void)));
    extern void        HAL_MPU_WriteByte(uint8_t I2Caddress, uint8_t regAddress,
                                         uint8_t data);
    extern void       HAL_MPU_WriteByteNB(uint8_t I2Caddress, uint8_t regAddress,
                                          uint8_t data);
    extern int8_t      HAL_MPU_ReadByte(uint8_t I2Caddress, uint8_t regAddress);
    extern void        HAL_MPU_ReadBytes(uint8_t I2Caddress, uint8_t regAddress,
                                         uint8_t count, uint8_t* dest);
    extern void        HAL_MPU_IntEnable(bool enable);
    extern bool        HAL_MPU_IntClear();
/**     MPU9250(timer) - related API    */
    extern void        HAL_TIM_Init();
    extern void        HAL_TIM_Start(uint32_t load);
    extern void        HAL_TIM_Stop();
    extern uint32_t    HAL_TIM_GetValue();

/**     TM4C peripheral - related API   */
    extern void        HAL_SetPWM(uint32_t id, uint32_t pwm);
    extern uint32_t    HAL_GetPWM(uint32_t id);

/**     TaskScheduler - related API     */
    extern uint8_t     HAL_TS_InitSysTick(uint32_t periodMs,
                                          void((*custHook)(void)));
    extern uint8_t     HAL_TS_StartSysTick();
    extern uint8_t     HAL_TS_StopSysTick();
    extern uint32_t    HAL_TS_GetTimeStepMS();

#ifdef __cplusplus
}
#endif
#endif /* TM4C1294_HAL_H_ */

