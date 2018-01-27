/**
 * hal_mpu_tm4c.c
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran
 */
#include "hal_mpu_tm4c.h"

#if defined(__HAL_USE_MPU9250_SPI__)       //  Compile only if module is enabled

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
#include "driverlib/ssi.h"
#include "driverlib/timer.h"


/**     MPU9250 - related macros        */
//#define MPU9250_I2C_BASE I2C2_BASE

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


    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_SSI2);

    GPIOPinConfigure(GPIO_PD3_SSI2CLK);
    GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);   //MISO
    GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);   //MOSI

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);

    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3);
    SSIConfigSetExpClk(SSI2_BASE, g_ui32SysClock, SSI_FRF_MOTO_MODE_0,
                           SSI_MODE_MASTER, 1000000, 8);
    SSIEnable(SSI2_BASE);

    uint32_t dummy[1];
    while (SSIDataGetNonBlocking(SSI2_BASE, &dummy[0]));


    //  Configure power-switch pin
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_4);
    MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 0x00);

    //  Configure interrupt pin to receive output
    //      (not used as actual interrupts)
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_5);
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);

    //  Configure slave select pin - PN2
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2);
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0xFF);

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
    uint32_t dummy[1];

    regAddress = regAddress & 0x7F; //  MSB = 0 for writing operation
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x00);
    HAL_DelayUS(1);
    SSIDataPut(SSI2_BASE, regAddress);
    SSIDataPut(SSI2_BASE, data);
    while(SSIBusy(SSI2_BASE));
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0xFF);

    while (SSIDataGetNonBlocking(SSI2_BASE, &dummy[0]));
}

/**
 * Write one byte of data to I2C bus (non-blocking)
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress address of register in I2C device to write into
 * @param data to write into the register of I2C device
 */
void HAL_MPU_WriteByteNB(uint8_t I2Caddress, uint8_t regAddress, uint8_t data)
{
    uint32_t dummy[1];

    regAddress = regAddress & 0x7F; //  MSB = 0 for writing operation
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x00);
    HAL_DelayUS(1);
    SSIDataPutNonBlocking(SSI2_BASE, regAddress);
    SSIDataPutNonBlocking(SSI2_BASE, data);
    while(SSIBusy(SSI2_BASE));
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0xFF);

    while (SSIDataGetNonBlocking(SSI2_BASE, &dummy[0]));
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
    uint32_t dummy[1];

    regAddress = regAddress & 0x7F; //  MSB = 0 for writing operation
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x00);
    HAL_DelayUS(1);
    SSIDataPut(SSI2_BASE, regAddress);

    for (i = 0; i < length; i++)
        SSIDataPut(SSI2_BASE, data[i]);

    while(SSIBusy(SSI2_BASE));
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0xFF);

    while (SSIDataGetNonBlocking(SSI2_BASE, &dummy[0]));

    return 0;
}

/**
 * Read one byte of data from I2C device (performs dummy write as well)
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress address of register in I2C device to write into
 * @return data received from I2C device
 */
uint8_t HAL_MPU_ReadByte(uint8_t I2Caddress, uint8_t regAddress)
{
    uint32_t dummy[1], data;

    regAddress = regAddress | 0x80; //  MSB = 1 for reading operation
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x00);
    HAL_DelayUS(1);

    SSIDataPut(SSI2_BASE, regAddress);
    SSIDataGet(SSI2_BASE, &dummy[0]);
    SSIDataPut(SSI2_BASE, 0x00);
    SSIDataGet(SSI2_BASE, &data);

    while(SSIBusy(SSI2_BASE));
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0xFF);

    while (SSIDataGetNonBlocking(SSI2_BASE, &dummy[0]));

    return (uint8_t)(data & 0xFF);
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
    uint32_t dummy[1];

    regAddress = regAddress | 0x80; //  MSB = 1 for reading operation
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x00);
    HAL_DelayUS(1);

    SSIDataPut(SSI2_BASE, regAddress);
    SSIDataGet(SSI2_BASE, &dummy[0]);

    for (i = 0; i < length; i++)
    {
        uint32_t tmp;
        SSIDataPut(SSI2_BASE, 0x00);
        SSIDataGet(SSI2_BASE, &tmp);
        data[i] = tmp & 0xFF;
    }

    while(SSIBusy(SSI2_BASE));
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0xFF);

    while (SSIDataGetNonBlocking(SSI2_BASE, &dummy[0]));

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
#endif /* __HAL_USE_MPU9250_SPI__ */
