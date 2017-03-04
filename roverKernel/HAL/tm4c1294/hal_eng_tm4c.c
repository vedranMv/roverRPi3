/*
 * hal_eng_tm4c.c
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran
 */
#include "hal_eng_tm4c.h"

#if defined(__HAL_USE_ENGINES__)

#include "roverKernel/libs/myLib.h"
#include "roverKernel/HAL/tm4c1294/hal_common_tm4c.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"


/**     Engines - related macros      */
#define ED_PWM_BASE     PWM0_BASE
#define ED_PWM_LEFT     PWM_OUT_2
#define ED_PWM_RIGHT    PWM_OUT_3
#define ED_HBR_BASE     GPIO_PORTL_BASE             //  H-bridge base
#define ED_HBR_PINL     (GPIO_PIN_0 | GPIO_PIN_1)   //  H-bridge left-wheel pins
#define ED_HBR_PINR     (GPIO_PIN_2 | GPIO_PIN_3)   //  H-bridge right-wheel pins

uint32_t g_ui32SysClock;

/**
 * Initialize hardware used to run engines
 * @param pwmMin pwm value used to stop the motors
 * @param pwmMax pwm value used to run motors at full speed
 */
void HAL_ENG_Init(uint32_t pwmMin, uint32_t pwmMax)
{
    /// Configuration and initialization of PWM for engines
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    /// Set PWM clock divider to 32 - allows for low frequencies
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_32);


    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinConfigure(GPIO_PF3_M0PWM3);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
    PWMGenConfigure(ED_PWM_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN |
                                          PWM_GEN_MODE_NO_SYNC);
    /// Target frequency ~60Hz (PWM generator clock -> 20kHZ)
    PWMGenPeriodSet(ED_PWM_BASE, PWM_GEN_1, pwmMax + 1);
    PWMPulseWidthSet(ED_PWM_BASE, ED_PWM_LEFT, pwmMin);
    PWMPulseWidthSet(ED_PWM_BASE, ED_PWM_RIGHT, pwmMin);
    PWMGenEnable(ED_PWM_BASE, PWM_GEN_1);   ///Stop PWM generator block.
    /// Disable the PWM1 output signal (PF1).
    PWMOutputState(ED_PWM_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, false);

    /// Configure H-bridges for each engine
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    GPIOPinTypeGPIOOutput(ED_HBR_BASE, ED_HBR_PINL | ED_HBR_PINR);
    GPIOPinWrite(ED_HBR_BASE, ED_HBR_PINL | ED_HBR_PINR, 0x00);

    /// Initialize optical encoders that track wheel movement - interrupt based
    /// Dont't forget to update ISR pointers in interrupt vector in startuo_ccs.c
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOP);
    GPIOPinTypeGPIOInput(GPIO_PORTP_BASE,GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE,GPIO_PIN_2);
    GPIOPadConfigSet(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_DIR_MODE_IN);

    GPIOIntEnable(GPIO_PORTP_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
    GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_0 | GPIO_PIN_1, GPIO_RISING_EDGE | GPIO_DISCRETE_INT);

    IntDisable(INT_GPIOP0);
    IntDisable(INT_GPIOP1);
}

/**
 * Enable/disable PWM output from the microcontroller
 * @param engine engine ID (one of ED_X macros from engines.h library)
 * @param enable state of PWM output
 */
void HAL_ENG_Enable(uint32_t engine, bool enable)
{
    if (engine == 0)
        PWMOutputState(ED_PWM_BASE, PWM_OUT_2_BIT , enable);
    else if (engine == 1)
        PWMOutputState(ED_PWM_BASE, PWM_OUT_3_BIT, enable);
    else if (engine == 2)
        PWMOutputState(ED_PWM_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, enable);
}

/**
 * Set PWM for engines provided in argument
 * @param engine is engine ID (one of ED_X macros from engines.h library)
 * @param pwm value to set PWM to (must be in range between 1 and max allowed)
 * @return HAL library error code
 */
uint8_t HAL_ENG_SetPWM(uint32_t engine, uint32_t pwm)
{
    if (pwm > PWMGenPeriodGet(ED_PWM_BASE, PWM_GEN_1))
        return HAL_ENG_PWMOOR;
    if (pwm < 1)
        return HAL_ENG_PWMOOR;

    if (engine == 0)
        PWMPulseWidthSet(ED_PWM_BASE, ED_PWM_LEFT, pwm);
    else if (engine == 1)
        PWMPulseWidthSet(ED_PWM_BASE, ED_PWM_RIGHT, pwm);
    else if (engine == 2)
    {
        PWMPulseWidthSet(ED_PWM_BASE, ED_PWM_LEFT, pwm);
        PWMPulseWidthSet(ED_PWM_BASE, ED_PWM_RIGHT, pwm);
    }
    else
        return HAL_ENG_EOOR;

    return HAL_OK;
}

uint32_t HAL_ENG_GetPWM(uint32_t engine)
{

    if (engine == 0)
        return PWMPulseWidthGet(ED_PWM_BASE, ED_PWM_LEFT);
    else if (engine == 1)
        return PWMPulseWidthGet(ED_PWM_BASE, ED_PWM_RIGHT);
    else
        return HAL_ENG_EOOR;
}

/**
 * Set H-bridge to specific state
 * @param mask mask of channels to configure
 * @param dir direction in which vehicle has to move
 * @return HAL library error code
 */
uint8_t HAL_ENG_SetHBridge(uint32_t mask, uint8_t dir)
{
    if (mask == 0)  /// Configure direction for left motor
        GPIOPinWrite(ED_HBR_BASE, ED_HBR_PINL, dir);
    else if (mask == 1) /// Configure direction for right motor
        GPIOPinWrite(ED_HBR_BASE, ED_HBR_PINR, dir);
    else if (mask == 2) /// Configure direction for both motors
        GPIOPinWrite(ED_HBR_BASE, ED_HBR_PINR | ED_HBR_PINL, dir);
    else
        return HAL_ENG_ILLM;

    return HAL_OK;
}

/**
 * Get current configuration of H-bridge
 * @param mask of channels for which to get the state
 * @return pin configuration of H-bridge for specific channel mask
 */
uint32_t HAL_ENG_GetHBridge(uint32_t mask)
{
    if (mask == 0)
        return GPIOPinRead(ED_HBR_BASE, ED_HBR_PINL);
    else if (mask == 1)
        return GPIOPinRead(ED_HBR_BASE, ED_HBR_PINR);
    else
        return GPIOPinRead(ED_HBR_BASE, ED_HBR_PINR | ED_HBR_PINL);
}

/**
 * Clear interrupt
 * @param engine is engine for which to clean interrupt
 */
void HAL_ENG_IntClear(uint32_t engine)
{
    if (engine == 0)
        GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_0);
    else if (engine == 1)
        GPIOIntClear(GPIO_PORTP_BASE,GPIO_INT_PIN_1);
}

/**
 * Change the state of an interrupt
 * @param engine for which to alter the state of interrupt
 * @param enable new state of interrupt
 */
void HAL_ENG_IntEnable(uint32_t engine, bool enable)
{
    if (engine == 0)
    {
        if (enable) IntEnable(INT_GPIOP0);
        else IntDisable(INT_GPIOP0);
    }

    if (engine == 1)
    {
        if (enable) IntEnable(INT_GPIOP1);
        else IntDisable(INT_GPIOP1);
    }
}

#endif  /* __HAL_USE_ENGINES__ */
