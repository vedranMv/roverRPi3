/**
 * hal_radar_tm4c.c
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran
 */
#include "hal_radar_tm4c.h"

#if defined(__HAL_USE_RADAR__)

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
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/systick.h"


/**     Radar - related macros        */
#define RADAR_PWM_BASE  PWM0_BASE
///  PWM pin of vertical & horizontal axis of radar
#define RAD_HORIZONTAL  0x00000041
#define RAD_VERTICAL    0x000000C4
///  PWM extremes of radar servos
#define RAD_MAX         59500       //Right/Up
#define RAD_MIN         54000       //Left/Down
#define RAD_PWM_ARG     62498       //PWM generator clock

/**
 * Initialize hardware used for IR radar peripheral
 */
void HAL_RAD_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

    //  Set PWM clock divider to 32 - allows for low frequencies
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_32);

    // Configuration and initialization of Radar left-right PWM output
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, RAD_PWM_ARG);   //~60Hz PWM clock
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, RAD_MIN+(RAD_MAX-RAD_MIN)/2);    //~10° for servo
    PWMGenEnable(PWM0_BASE, PWM_GEN_0); //Start PWM generator block
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);

    // Configuration and initialization of Radar up-down axis PWM output
    GPIOPinConfigure(GPIO_PG0_M0PWM4);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, RAD_PWM_ARG);   //~60Hz PWM clock
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,  RAD_MIN+(RAD_MAX-RAD_MIN)/2);    //~10° for servo
    PWMGenEnable(PWM0_BASE, PWM_GEN_2); //Start PWM generator block
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);

    // Configure Radar ADC module to sample AIN 3 (used to read IR sensor output)
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);//  Configure GPIO as ADC input
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                             ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
    //  Configure hardware averaging of 64 samples
    ADCHardwareOversampleConfigure(ADC0_BASE, 64);

    HAL_RAD_Enable(true);

    //  Wait for gimbal servos to get to specified positions
    HAL_DelayUS(500000);
}

/**
 * Enable/disable radar servos
 * @note Disabling servos disables PWM output leaving servos susceptible to any
 * noise on input possibly causing twitching of the joints
 * @param enable
 */
void HAL_RAD_Enable(bool enable)
{
    PWMOutputState(PWM0_BASE, RAD_HORIZONTAL, enable);
    PWMOutputState(PWM0_BASE, RAD_VERTICAL, enable);
}

/**
 * Set vertical(up-down) angle of radar gimbal
 * @param angle to move vertical joint to; 0°(up) to 160° (down)
 */
void HAL_RAD_SetVerAngle(float angle)
{
    float arg = finterpolatef(0.0f, (float)RAD_MIN, 160.0f, (float)RAD_MAX, angle);

    if ((angle > 160.0f) || (angle < 0.0f)) return;
    HAL_SetPWM(RAD_VERTICAL, (uint32_t)arg);
}

/**
 * Get current vertical(up-down) angle of radar gimbal
 * @return angle in range of 0° to 160°
 */
float HAL_RAD_GetVerAngle()
{
    float retVal = (float)HAL_GetPWM(RAD_VERTICAL);

    retVal = finterpolatef((float)RAD_MIN, 0, (float)RAD_MAX, 160, retVal);
    return retVal;
}

/**
 * Set horizontal(left-right) angle of radar gimbal
 * @param angle to move horizontal joint to; 0°(right) to 160° (left)
 */
void HAL_RAD_SetHorAngle(float angle)
{
    float arg = finterpolatef(0.0f, (float)RAD_MAX, 160.0f, (float)RAD_MIN, angle);

    if ((angle > 160.0f) || (angle < 0.0f)) return;
    HAL_SetPWM(RAD_HORIZONTAL, (uint32_t)arg);
}

/**
 * Get current horizontal angle of radar gimbal
 * @return angle in range of 0° to 160°
 */
float HAL_RAD_GetHorAngle()
{
    float retVal = (float)HAL_GetPWM(RAD_HORIZONTAL);

    retVal = finterpolatef((float)RAD_MAX, 0, (float)RAD_MIN, 160, retVal);
    return retVal;
}

/**
 * Read value from analog output of IR distance sensor
 * @return 12-bit analog value of distance measured by IR sensor
 */
uint32_t HAL_RAD_ADCTrigger()
{
    uint32_t retVal;
    ADCProcessorTrigger(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false));
    ADCIntClear(ADC0_BASE, 3);  //Clear flag
    ADCSequenceDataGet(ADC0_BASE, 3, &retVal);

    return retVal;
}

#endif  /* __HAL_USE_RADAR__ */
