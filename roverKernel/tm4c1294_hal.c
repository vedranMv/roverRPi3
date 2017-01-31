/**
 * tm4c1294_hal.c
 *
 *  Created on: 07. 08. 2016.
 *      Author: Vedran Mikov
 */
#include <stdbool.h>
#include <stdint.h>

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
#include "driverlib/adc.h"
#include "driverlib/i2c.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"
#include "driverlib/fpu.h"

#include "tm4c1294_hal.h"
#include "myLib.h"

uint32_t g_ui32SysClock;

/**     Engines - related macros      */
#define ED_PWM_BASE     PWM0_BASE
#define ED_PWM_LEFT     PWM_OUT_2
#define ED_PWM_RIGHT    PWM_OUT_3
#define ED_HBR_BASE     GPIO_PORTL_BASE             //  H-bridge base
#define ED_HBR_PINL     (GPIO_PIN_0 | GPIO_PIN_1)   //  H-bridge left-wheel pins
#define ED_HBR_PINR     (GPIO_PIN_2 | GPIO_PIN_3)   //  H-bridge right-wheel pins

/**     Radar - related macros        */
#define RADAR_PWM_BASE  PWM0_BASE
///  PWM pin of vertical & horizontal axis of radar
#define RAD_HORIZONTAL  0x00000041
#define RAD_VERTICAL    0x000000C4
///  PWM extremes of radar servos
#define RAD_MAX         59500       //Right/Up
#define RAD_MIN         54000       //Left/Down
#define RAD_PWM_ARG     62498       //PWM generator clock

/**     ESP8266 - related macros        */
#define ESP8266_UART_BASE UART7_BASE

/**     MPU9250 - related macros        */
#define MPU9250_I2C_BASE I2C2_BASE

///-----------------------------------------------------------------------------
///                                                                    [PRIVATE]
///-----------------------------------------------------------------------------
/**
 * Calculate load value from timer based on desired time in milliseconds
 * @param ms time in milliseconds
 * @return equivalent number of clock cycles for main oscillator
 */
uint32_t _TM4CMsToCycles(uint32_t ms)
{
    return (ms*(g_ui32SysClock/1000));
}


///-----------------------------------------------------------------------------
///                                                                     [PUBLIC]
///-----------------------------------------------------------------------------
/**
 * Initialize microcontroller board clock & enable on-board floating-point unit
 */
void HAL_BOARD_CLOCK_Init()
{
    // Set the clock to use on-board 25MHz oscillator and generate 120MHz clock
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                        SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                        SYSCTL_CFG_VCO_480), 120000000);
    //  Enable Floating-point unit (FPU)
    FPUEnable();
    FPULazyStackingEnable();
    //  Enable interrupt handler
    IntMasterEnable();
}

/**
 * Wait for given amount of us - blocking function
 * @param us time in us to wait
 */
void HAL_DelayUS(uint32_t us)   //1000
{
    float f = 1000000 / (float)us;//  Frequency = 1 / Period

    f = (float)g_ui32SysClock / (3.0 * f);
    SysCtlDelay((uint32_t)f);
}

/******************************************************************************
 ******************************************************************************
 ************               ESP8266 - related API                  ************
 ******************************************************************************
 ******************************************************************************/

/**
 * Initialize UART port communicating with ESP8266 chip - 8 data bits, no parity,
 * 1 stop bit, no flow control
 * @param baud designated speed of communication
 * @return HAL library error code
 */
uint32_t HAL_ESP_InitPort(uint32_t baud)
{
    static bool pinInit = false;

    //  Prevents reinitializing the pins every time a baud rate is updated
    if (!pinInit)
    {
        //  Configure HW pins for UART and on/off signal
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
        GPIOPinConfigure(GPIO_PC4_U7RX);
        GPIOPinConfigure(GPIO_PC5_U7TX);
        GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

        GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00);

        GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF);

        pinInit = true;
    }

    //    Configure UART 7 peripheral used for ESP communication
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    SysCtlPeripheralReset(SYSCTL_PERIPH_UART7);
    UARTClockSourceSet(ESP8266_UART_BASE, UART_CLOCK_SYSTEM);
    UARTConfigSetExpClk(ESP8266_UART_BASE, g_ui32SysClock, baud,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
    UARTEnable(ESP8266_UART_BASE);
    HAL_DelayUS(50000);    //  50ms delay after configuring

    return HAL_OK;
}

/**
 * Attach specific interrupt handler to ESP's UART and configure interrupt to
 * occur on every received character
 */
void HAL_ESP_RegisterIntHandler(void((*intHandler)(void)))
{
    UARTDisable(ESP8266_UART_BASE);
    //  Enable Interrupt on received data
    UARTFIFOLevelSet(ESP8266_UART_BASE,UART_FIFO_TX1_8, UART_FIFO_RX1_8 );
    ///UARTFIFODisable(ESP8266_UART_BASE);
    UARTIntRegister(ESP8266_UART_BASE, intHandler);
    UARTIntEnable(ESP8266_UART_BASE, UART_INT_RX | UART_INT_RT);
    IntDisable(INT_UART7);
    UARTEnable(ESP8266_UART_BASE);
}

/**
 * Enable or disable ESP chip by toggling a pin connected to CH_PD
 * @param enable is state of device
 */
void HAL_ESP_HWEnable(bool enable)
{
    ///    After both actions add a delay to allow chip to settle
    if (!enable)
    {
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
        SysCtlDelay(g_ui32SysClock/3);
        HAL_DelayUS(1000000);
    }
    else
    {
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF);
        HAL_DelayUS(2000000);
    }
}

/**
 * Check whether the chip is enabled or disabled
 */
bool HAL_ESP_IsHWEnabled()
{
    return ((GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6) & GPIO_PIN_6) > 0);
}

/**
 * Enable/disable UART interrupt - interrupt occurs on every char received
 * @param enable
 */
void HAL_ESP_IntEnable(bool enable)
{
    if (enable) IntEnable(INT_UART7);
    else IntDisable(INT_UART7);
}

/**
 * Clear all interrupt flags when an interrupt occurs
 */
int32_t HAL_ESP_ClearInt()
{
    uint32_t retVal = UARTIntStatus(ESP8266_UART_BASE, true);
    //  Clear all raised interrupt flags
    UARTIntClear(ESP8266_UART_BASE, retVal);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, ~GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1));
    return retVal;
}

/**
 * Check if UART port is busy at the moment
 * @return true: port is currently busy
 *        false: port is free for starting communication
 */
bool HAL_ESP_UARTBusy()
{
    return UARTBusy(ESP8266_UART_BASE);
}

/**
 * Send single char over UART
 * @param arg character to send
 */
void HAL_ESP_SendChar(char arg)
{
    UARTCharPut(ESP8266_UART_BASE, arg);
}

/**
 * Check if there are any characters available in UART RX buffer
 * @return true: there are characters in RX buffer
 *        false: no characters in RX buffer
 */
bool HAL_ESP_CharAvail()
{
    return UARTCharsAvail(ESP8266_UART_BASE);
}

/**
 * Get single character from UART RX buffer
 * @return first character in RX buffer
 */
char  HAL_ESP_GetChar()
{
    return UARTCharGetNonBlocking(ESP8266_UART_BASE);
}

/**
 * Watchdog timer for ESP module - used to reset protocol if communication hangs
 * for too long
 */
void HAL_ESP_InitWD(void((*intHandler)(void)))
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER6);
    TimerConfigure(TIMER6_BASE, TIMER_CFG_ONE_SHOT_UP);
    TimerIntRegister(TIMER6_BASE, TIMER_A, intHandler);
    TimerIntEnable(TIMER6_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER6A);
}


void HAL_ESP_WDControl(bool enable, uint32_t ms)
{
    //  Record last value for timeout, use it when timeout argument is 0
    static uint32_t LTM;
    if (enable)
    {
        if (ms != 0)
        {
            TimerLoadSet(TIMER6_BASE, TIMER_A, _TM4CMsToCycles(ms));
            LTM = ms;
        }
        else
            TimerLoadSet(TIMER6_BASE, TIMER_A, _TM4CMsToCycles(LTM));
        HWREG(TIMER6_BASE + TIMER_O_TAV) = 0;
        TimerEnable(TIMER6_BASE, TIMER_A);
    }
    else
        TimerDisable(TIMER6_BASE, TIMER_A);
}

void HAL_ESP_WDClearInt()
{
    TimerIntClear(TIMER6_BASE, TimerIntStatus(TIMER6_BASE, true));
    IntPendSet(INT_UART7);
}

/******************************************************************************
 ******************************************************************************
 ************               Engines - related API                  ************
 ******************************************************************************
 ******************************************************************************/

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
    GPIOPadConfigSet(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_DIR_MODE_IN);

    GPIOIntEnable(GPIO_PORTP_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
    GPIOIntTypeSet(GPIO_PORTP_BASE,GPIO_PIN_0 | GPIO_PIN_1, GPIO_RISING_EDGE | GPIO_DISCRETE_INT);

    IntDisable(INT_GPIOP0);
    IntDisable(INT_GPIOP1);
}

/**
 * Enable/disable PWM output from the microcontroller
 * @param enable state of PWM output
 */
void HAL_ENG_Enable(bool enable)
{
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

/******************************************************************************
 ******************************************************************************
 ************               Radar - related API                    ************
 ******************************************************************************
 ******************************************************************************/

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

    ///  Set PWM clock divider to 32 - allows for low frequencies
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_32);

    /// Configuration and initialization of Radar left-right PWM output
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, RAD_PWM_ARG);   //~60Hz PWM clock
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, RAD_MIN+(RAD_MAX-RAD_MIN)/2);    //~10° for servo
    PWMGenEnable(PWM0_BASE, PWM_GEN_0); //Start PWM generator block
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);

    /// Configuration and initialization of Radar up-down axis PWM output
    GPIOPinConfigure(GPIO_PG0_M0PWM4);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, RAD_PWM_ARG);   //~60Hz PWM clock
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,  RAD_MIN+(RAD_MAX-RAD_MIN)/2);    //~10° for servo
    PWMGenEnable(PWM0_BASE, PWM_GEN_2); //Start PWM generator block
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);

    /// Configure Radar ADC module to sample AIN 3 (used to read IR sensor output)
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);//Configure GPIO as ADC input
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                             ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
    //Configure hardware averaging of 64 samples
    ADCHardwareOversampleConfigure(ADC0_BASE, 64);

    HAL_RAD_Enable(true);

    ///  Wait for gimbal servos to get to specified positions
    SysCtlDelay(g_ui32SysClock/2);
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

/******************************************************************************
 ******************************************************************************
 ************               MPU9250 - related API                  ************
 ******************************************************************************
 ******************************************************************************/

/**
 * Initializes I2C 2 bus for communication with MPU (SDA - PN4, SCL - PN5)
 *      Bus frequency 1MHz, connection timeout: 100ms
 * Initializes interrupt pin(PA5) that will be toggled by MPU9250
 *      when it has data available for reading
 *      -PA5 is push-pull pin with weak pull down and 10mA strength
 */
void HAL_MPU_Init(void((*custHook)(void)))
{
    uint32_t ui32TPR;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

    /// Enable I2C communication interface, SCL, SDA lines
    GPIOPinConfigure(GPIO_PN4_I2C2SDA);
    GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);

    I2CMasterEnable(MPU9250_I2C_BASE);

    /// Run I2C bus on 1MHz custom clock
    I2CMasterInitExpClk(MPU9250_I2C_BASE, g_ui32SysClock, true);

    //  Taken from TivaWare library!
    //
    // Compute the clock divider that achieves the fastest speed less than or
    // equal to the desired speed.  The numerator is biased to favor a larger
    // clock divider so that the resulting clock is always less than or equal
    // to the desired clock, never greater.
    //
    ui32TPR = ((120000000 + (2 * 10 * 1000000) - 1) / (2 * 10 * 1000000)) - 1;
    HWREG(MPU9250_I2C_BASE + 0x00C) = ui32TPR;
    while (I2CMasterBusy(MPU9250_I2C_BASE));
    I2CMasterTimeoutSet(MPU9250_I2C_BASE, g_ui32SysClock/10);

    ///  Configure interrupt pin to have weak pull down, 10mA strength
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_5);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_STRENGTH_10MA, GPIO_PIN_TYPE_STD_WPD);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);

    ///  Set up an interrupt, and interrupt handler
    if (custHook != 0)
    {
        GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_INT_PIN_5, GPIO_FALLING_EDGE);
        GPIOIntRegister(GPIO_PORTA_BASE, custHook);
        GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_5);
        IntDisable(INT_GPIOA);
    }


    HAL_TIM_Init();
}
/**
 * Write one byte of data to I2C bus and wait until transmission is over (blocking)
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress address of register in I2C device to write into
 * @param data to write into the register of I2C device
 */
void HAL_MPU_WriteByte(uint8_t I2Caddress, uint8_t regAddress, uint8_t data)
{
    I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, false);
    I2CMasterDataPut(MPU9250_I2C_BASE, regAddress);
    I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(!I2CMasterBusy(MPU9250_I2C_BASE));
    while(I2CMasterBusy(MPU9250_I2C_BASE));

    I2CMasterDataPut(MPU9250_I2C_BASE, data);
    I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

    while(!I2CMasterBusy(MPU9250_I2C_BASE));
    while(I2CMasterBusy(MPU9250_I2C_BASE));
}

/**
 * Write one byte of data to I2C bus (non-blocking)
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress address of register in I2C device to write into
 * @param data to write into the register of I2C device
 */
void HAL_MPU_WriteByteNB(uint8_t I2Caddress, uint8_t regAddress, uint8_t data)
{
    I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, false);
    I2CMasterDataPut(MPU9250_I2C_BASE, regAddress);
    I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(!I2CMasterBusy(MPU9250_I2C_BASE));
    while(I2CMasterBusy(MPU9250_I2C_BASE));

    I2CMasterDataPut(MPU9250_I2C_BASE, data);
    I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
}

/**
 * Read one byte of data from I2C device (performs dummy write as well)
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress address of register in I2C device to write into
 * @return data received from I2C device
 */
int8_t HAL_MPU_ReadByte(uint8_t I2Caddress, uint8_t regAddress)
{
    uint32_t data, dummy = 0;
    dummy = dummy+0;//Put to avoid warning about dummy not being used

    I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, false);
    I2CMasterDataPut(MPU9250_I2C_BASE, regAddress);
    I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(!I2CMasterBusy(MPU9250_I2C_BASE));
    while(I2CMasterBusy(MPU9250_I2C_BASE));

    I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, true);
    I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(!I2CMasterBusy(MPU9250_I2C_BASE));
    while(I2CMasterBusy(MPU9250_I2C_BASE));
    data = I2CMasterDataGet(MPU9250_I2C_BASE);

    I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(!I2CMasterBusy(MPU9250_I2C_BASE));
    while(I2CMasterBusy(MPU9250_I2C_BASE));
    dummy = I2CMasterDataGet(MPU9250_I2C_BASE);

    return (data & 0xFF);
}

/**
 * Read several bytes from I2C device (performs dummy write as well)
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress address of register in I2C device to write into
 * @param count number of bytes to red
 * @param dest pointer to data buffer in which data is saved after reading
 */
void HAL_MPU_ReadBytes(uint8_t I2Caddress, uint8_t regAddress,
                       uint8_t count, uint8_t* dest)
{
    uint8_t i;
    for (i = 0; i < count; i++)
    {
        *(dest + i) = HAL_MPU_ReadByte(I2Caddress, regAddress + i);
    }
}

/**
 * Enable pin interrupt used by MPU to signal it has new data available. If
 * interrupt is enabled means a data is expected therefore a timer is started as
 * well to measure time interval between two sensor measurements.
 * @param enable boolean value with new state (1-enable or 0-disable)
 */
void HAL_MPU_IntEnable(bool enable)
{
    if (enable)
    {
        HAL_TIM_Start(0);
        IntEnable(INT_GPIOA);
    }
    else
    {
        HAL_TIM_Stop();
        IntDisable(INT_GPIOA);
    }
}

/**
 * Clear all raised interrupts on port A (pin interrupt by MPU) and return true
 * if interrupt occurred on pin 5 (used by MPU)
 * @return true: if interrupt was raised by pin 5 on port A
 *        false: otherwise
 */
bool HAL_MPU_IntClear()
{
    uint32_t intStat = GPIOIntStatus(GPIO_PORTA_BASE, true);
    GPIOIntClear(GPIO_PORTA_BASE, intStat);

    if ( (intStat & (GPIO_INT_PIN_5)) != GPIO_INT_PIN_5) return false;
    else return true;
}

/**
 * Initialize timer used to precisely measure time interval between two sensor
 * measurements (dt constant used for integration of MPU data)
 */
void HAL_TIM_Init()
{
    /// Set up timer to measure period between two sensor measurements
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER7);
    TimerConfigure(TIMER7_BASE, TIMER_CFG_ONE_SHOT_UP);
    /// Set up some big load that should never be reached (dT usually < 1s)
    TimerLoadSet(TIMER7_BASE, TIMER_A, 2*g_ui32SysClock);
}

/**
 * Start the timer to count up from a specified load value
 * @param load desired load value from which to start counting up
 */
void HAL_TIM_Start(uint32_t load)
{
    ///  Set load for Timer7, timer A
    HWREG(TIMER7_BASE + TIMER_O_TAV) = load;
    ///  Start timer
    TimerEnable(TIMER7_BASE, TIMER_A);
}

/**
 * Stop timer
 */
void HAL_TIM_Stop()
{
    TimerDisable(TIMER7_BASE, TIMER_A);
}

/**
 * Get current value of the timer's internal counter
 * @return value of timer's internal counter
 */
uint32_t HAL_TIM_GetValue()
{
    return TimerValueGet(TIMER7_BASE, TIMER_A);
}

/******************************************************************************
 ******************************************************************************
 ************           TM4C peripheral - related API              ************
 ******************************************************************************
 ******************************************************************************/

/**
 * Set desired PWM duty cycle on specific output channel
 * @param id is channel ID of PWM channel affected
 * @param pwm value of PWM pulse (duty cycle) to set
 */
void HAL_SetPWM(uint32_t id, uint32_t pwm)
{
    PWMPulseWidthSet(PWM0_BASE, id, pwm);
}
/**
 * Get current PWM duty cycle on specific output channel
 * @param id is channel ID of PWM channel affected
 * @return PWM duty cycle at channel id
 */
uint32_t HAL_GetPWM(uint32_t id)
{
    return PWMPulseWidthGet(PWM0_BASE, id);
}

/******************************************************************************
 ******************************************************************************
 ************            TaskScheduler - related API               ************
 ******************************************************************************
 ******************************************************************************/

/**
 * Setup SysTick interrupt and period
 * @param periodMs time in milliseconds how often to trigger an interrupt
 * @param custHook pointer to function that will be called on SysTick interrupt
 * @return HAL library error code
 */
///Keep track whether the SysTick has already been configured
bool _systickSet = false;
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

    SysTickPeriodSet(periodMs*(g_ui32SysClock/1000));
    SysTickIntRegister(custHook);
    SysTickIntEnable();
    _systickSet = true;

    return 0;
}

/**
 * Wrapper for SysTick start function
 */
uint8_t HAL_TS_StartSysTick()
{
    if(_systickSet)
        SysTickEnable();
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
        SysTickDisable();
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
    return (SysTickPeriodGet()*1000)/g_ui32SysClock;
}

