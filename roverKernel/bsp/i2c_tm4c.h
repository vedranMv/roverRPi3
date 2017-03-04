/**
 *  spi.h
 *
 *  Created on: 4. 3. 2017.
 *      Author: Vedran Mikov
 *
 *  This library provides common interface between hardware peripherals and
 *  underlying hardware or manufacturer's middleware.
 */

#ifndef TM4C1294NCPTD_I2C_
#define TM4C1294NCPTD_I2C_

#ifdef __cplusplus
extern "C"
{
#endif

/*      I2C 2 peripheral    */
void    BSP_I2C2Init();
int8_t  BSP_I2C2Write();
int8_t  BSP_I2C2Read();

#ifdef __cplusplus
}
#endif
#endif /* TM4C1294NCPTD_SPI_ */
