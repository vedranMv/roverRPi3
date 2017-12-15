/**
 * hal_mpu_tm4c.h
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran Mikov
 *
 ****Hardware dependencies:
 *      I2C2 - Communication with sensor, pins PN4(SDA) & PN5(SCL)
 *      GPIO PA5 - Data available interrupt pin (normal input, not interrupt pin)
 *      GPIO PL4 - Power switch for MPU (active high)
 */
#include "hwconfig.h"

//  Compile following section only if hwconfig.h says to include this module
#if !defined(ROVERKERNEL_HAL_TM4C1294_HAL_MPU_TM4C_H_) && defined(__HAL_USE_MPU9250__)
#define ROVERKERNEL_HAL_TM4C1294_HAL_MPU_TM4C_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**     MPU9250(I2C) - related HW API       */
    extern void        HAL_MPU_Init();
    extern void        HAL_MPU_PowerSwitch(bool powerState);
    extern void        HAL_MPU_WriteByte(uint8_t I2Caddress, uint8_t regAddress,
                                         uint8_t data);
    extern void        HAL_MPU_WriteByteNB(uint8_t I2Caddress, uint8_t regAddress,
                                          uint8_t data);
    extern uint8_t     HAL_MPU_WriteBytes(uint8_t I2Caddress, uint8_t regAddress,
                                          uint16_t length, uint8_t *data);
    extern int8_t      HAL_MPU_ReadByte(uint8_t I2Caddress, uint8_t regAddress);
    extern uint8_t     HAL_MPU_ReadBytes(uint8_t I2Caddress, uint8_t regAddress,
                                         uint16_t length, uint8_t* data);
    extern bool        HAL_MPU_DataAvail();

#ifdef __cplusplus
}
#endif

#endif /* ROVERKERNEL_HAL_TM4C1294_HAL_MPU_TM4C_H_ */
