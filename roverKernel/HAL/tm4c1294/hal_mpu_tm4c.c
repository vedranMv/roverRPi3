/**
 * hal_mpu_tm4c.c
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran
 */
#include "hal_mpu_tm4c.h"

#if defined(__HAL_USE_MPU9250__)       //  Compile only if module is enabled

#include "libs/myLib.h"
#include "HAL/tm4c1294/hal_common_tm4c.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"

#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
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

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);

    // Enable I2C communication interface, SCL, SDA lines
    MAP_GPIOPinConfigure(GPIO_PN4_I2C2SDA);
    MAP_GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    MAP_GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    MAP_GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);

    MAP_I2CMasterEnable(MPU9250_I2C_BASE);

    // Run I2C bus on 1MHz custom clock
    MAP_I2CMasterInitExpClk(MPU9250_I2C_BASE, g_ui32SysClock, true);

    //  Configure power-switch pin
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_4);
    MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 0x00);

    //  Configure interrupt pin to receive output
    //      (not used as actual interrupts)
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_5);
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);

}

/**
 * Control power-switch for MPU9250
 * Controls whether or not MPU sensors receives power (n-ch MOSFET as switch)
 * @param powerState desired state of power switch (active high)
 */
void HAL_MPU_PowerSwitch(bool powerState)
{
    if (powerState)
        MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 0xFF);
    else
        MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 0x00);
}
/**
 * Write one byte of data to I2C bus and wait until transmission is over (blocking)
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress address of register in I2C device to write into
 * @param data to write into the register of I2C device
 */
void HAL_MPU_WriteByte(uint8_t I2Caddress, uint8_t regAddress, uint8_t data)
{
    MAP_I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, false);
    MAP_I2CMasterDataPut(MPU9250_I2C_BASE, regAddress);
    MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));

    MAP_I2CMasterDataPut(MPU9250_I2C_BASE, data);
    MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));
}

/**
 * Write one byte of data to I2C bus (non-blocking)
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress address of register in I2C device to write into
 * @param data to write into the register of I2C device
 */
void HAL_MPU_WriteByteNB(uint8_t I2Caddress, uint8_t regAddress, uint8_t data)
{
    MAP_I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, false);
    MAP_I2CMasterDataPut(MPU9250_I2C_BASE, regAddress);
    MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));

    MAP_I2CMasterDataPut(MPU9250_I2C_BASE, data);
    MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
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
    MAP_I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, false);
    MAP_I2CMasterDataPut(MPU9250_I2C_BASE, regAddress);
    MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));

    for (i = 0; i < (length-1); i++)
    {
        MAP_I2CMasterDataPut(MPU9250_I2C_BASE, data[i]);
        MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        HAL_DelayUS(4);
        while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));
    }

    MAP_I2CMasterDataPut(MPU9250_I2C_BASE, data[length-1]);
    MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));

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

    MAP_I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, false);
    MAP_I2CMasterDataPut(MPU9250_I2C_BASE, regAddress);
    MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));

    MAP_I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, true);
    MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));
    data = MAP_I2CMasterDataGet(MPU9250_I2C_BASE);

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

    MAP_I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, false);
    MAP_I2CMasterDataPut(MPU9250_I2C_BASE, regAddress);
    MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));

    MAP_I2CMasterSlaveAddrSet(MPU9250_I2C_BASE, I2Caddress, true);
    if (length == 1)
        MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    else
    {
        MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        HAL_DelayUS(4);
        while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));
        data[0] = (uint8_t)(MAP_I2CMasterDataGet(MPU9250_I2C_BASE) & 0xFF);

        for (i = 1; i < (length-1); i++)
        {
            MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            HAL_DelayUS(4);
            while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));
            data[i] = (uint8_t)(MAP_I2CMasterDataGet(MPU9250_I2C_BASE) & 0xFF);
        }
        MAP_I2CMasterControl(MPU9250_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    }

    HAL_DelayUS(4);
    while(MAP_I2CMasterBusy(MPU9250_I2C_BASE));
    data[length-1] = (uint8_t)(MAP_I2CMasterDataGet(MPU9250_I2C_BASE) & 0xFF);

    return 0;
}

/**
 * Check if MPU has raised interrupt to notify it has new data ready
 * @return true if interrupt pin is active, false otherwise
 */
bool HAL_MPU_DataAvail()
{
    return (MAP_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5) != 0);
}
#endif /* __HAL_USE_MPU9250__ */
