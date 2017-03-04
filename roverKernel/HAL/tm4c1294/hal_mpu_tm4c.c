/**
 * hal_mpu_tm4c.c
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran
 */
#include "hal_mpu_tm4c.h"

#if defined(__HAL_USE_MPU9250__)       //  Compile only if module is enabled

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
#include "utils/uartstdio.h"
#include "driverlib/i2c.h"
#include "driverlib/timer.h"


/**     MPU9250 - related macros        */
#define MPU9250_I2C_BASE I2C2_BASE

/**
 * Initializes I2C 2 bus for communication with MPU (SDA - PN4, SCL - PN5)
 *      Bus frequency 1MHz, connection timeout: 100ms
 * Initializes interrupt pin(PA5) that will be toggled by MPU9250
 *      when it has data available for reading
 *      -PA5 is push-pull pin with weak pull down and 10mA strength
 */

void HAL_MPU_Init(void((*custHook)(void)))
{
    //uint32_t ui32TPR;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

    // Enable I2C communication interface, SCL, SDA lines
    GPIOPinConfigure(GPIO_PN4_I2C2SDA);
    GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);

    I2CMasterEnable(MPU9250_I2C_BASE);

    // Run I2C bus on 1MHz custom clock
    I2CMasterInitExpClk(MPU9250_I2C_BASE, g_ui32SysClock, true);
    //I2CMasterGlitchFilterConfigSet(MPU9250_I2C_BASE, I2C_MASTER_GLITCH_FILTER_8);

    /*//  Taken from TivaWare library!
    //
    // Compute the clock divider that achieves the fastest speed less than or
    // equal to the desired speed.  The numerator is biased to favor a larger
    // clock divider so that the resulting clock is always less than or equal
    // to the desired clock, never greater.
    //
    ui32TPR = ((120000000 + (2 * 10 * 1000000) - 1) / (2 * 10 * 1000000)) - 1;
    HWREG(MPU9250_I2C_BASE + 0x00C) = ui32TPR;
    while (I2CMasterBusy(MPU9250_I2C_BASE));
    I2CMasterTimeoutSet(MPU9250_I2C_BASE, g_ui32SysClock/10);*/

    //  Configure interrupt pin to receive output
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_5);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);

    //  Set up an interrupt, and interrupt handler
    if (custHook != 0)
    {
        GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_INT_PIN_5, GPIO_FALLING_EDGE);
        //  GPIOInterRegister internally calls IntEnable() function causing
        //  interrupt to trigger. Prevent this by first disabling interrupt
        //  with GPIOIntDis
        //userHook = custHook;
        GPIOIntDisable(GPIO_PORTA_BASE, GPIO_INT_PIN_5);
        GPIOIntRegister(GPIO_PORTA_BASE, custHook);

        IntDisable(INT_GPIOA);
        GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_5);
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
    HAL_DelayUS(4);
    while(I2CMasterBusy(MPU9250_I2C_BASE));

    I2CMasterDataPut(MPU9250_I2C_BASE, data);
    I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    HAL_DelayUS(4);
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
    HAL_DelayUS(4);
    while(I2CMasterBusy(MPU9250_I2C_BASE));

    I2CMasterDataPut(MPU9250_I2C_BASE, data);
    I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
}

/**
 * Send a byte-array of data through I2C bus(blocking)
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress address of register in I2C device to write into
 * @param data buffer of data to send
 * @param length of data to send
 */
uint8_t HAL_MPU_WriteBytes(uint8_t I2Caddress, uint8_t regAddress,  uint16_t length, uint8_t *data)
{
    uint16_t i;
    I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, false);
    I2CMasterDataPut(MPU9250_I2C_BASE, regAddress);
    I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    HAL_DelayUS(4);
    while(I2CMasterBusy(MPU9250_I2C_BASE));

    for (i = 0; i < (length-1); i++)
    {
        I2CMasterDataPut(MPU9250_I2C_BASE, data[i]);
        I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        HAL_DelayUS(4);
        while(I2CMasterBusy(MPU9250_I2C_BASE));
    }

    I2CMasterDataPut(MPU9250_I2C_BASE, data[length-1]);
    I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    HAL_DelayUS(4);
    while(I2CMasterBusy(MPU9250_I2C_BASE));

    return 0;
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
    UNUSED(dummy);  //Prevent unused-variable warning

    I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, false);
    I2CMasterDataPut(MPU9250_I2C_BASE, regAddress);
    I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    HAL_DelayUS(4);
    while(I2CMasterBusy(MPU9250_I2C_BASE));

    I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, true);
    I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    HAL_DelayUS(4);
    while(I2CMasterBusy(MPU9250_I2C_BASE));
    data = I2CMasterDataGet(MPU9250_I2C_BASE);

    return (data & 0xFF);
}

/**
 * Read several bytes from I2C device (performs dummy write as well)
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress address of register in I2C device to write into
 * @param count number of bytes to red
 * @param dest pointer to data buffer in which data is saved after reading
 */
uint8_t HAL_MPU_ReadBytes(uint8_t I2Caddress, uint8_t regAddress,
                       uint16_t length, uint8_t* data)
{
    uint16_t i;

    I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, false);
    I2CMasterDataPut(MPU9250_I2C_BASE, regAddress);
    I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    HAL_DelayUS(4);
    while(I2CMasterBusy(MPU9250_I2C_BASE));

    I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, true);
    if (length == 1)
        I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    else
    {
        I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        HAL_DelayUS(4);
        while(I2CMasterBusy(MPU9250_I2C_BASE));
        data[0] = (uint8_t)(I2CMasterDataGet(MPU9250_I2C_BASE) & 0xFF);

        for (i = 1; i < (length-1); i++)
        {
            I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            HAL_DelayUS(4);
            while(I2CMasterBusy(MPU9250_I2C_BASE));
            data[i] = (uint8_t)(I2CMasterDataGet(MPU9250_I2C_BASE) & 0xFF);
        }
        I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    }

    HAL_DelayUS(4);
    while(I2CMasterBusy(MPU9250_I2C_BASE));
    data[length-1] = (uint8_t)(I2CMasterDataGet(MPU9250_I2C_BASE) & 0xFF);

    return 0;
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
/*
void HAL_MPU_ISR(void)
{
    HAL_MPU_IntClear();
    if (userHook != 0)
        userHook();
}*/

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

#endif /* __HAL_USE_MPU9250__ */
