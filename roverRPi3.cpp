#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"
#include "driverlib/ssi.h"

#include "roverKernel/rpiDriver.h"
#include "roverKernel/tm4c1294_hal.h"

#include "roverKernel/uartHW.h"
#include "roverKernel/taskScheduler.h"

//#define __DEBUG_SESSION__

//RPIRover rpiRov(6.8f, 14.3f, 25.0f, 40);

uint8_t taskID;
float   taskArgs[3];

void LEDCallback()
{
    //  Switch based on the current task
    switch(taskID)
    {
    case 0:
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0xFF);
        break;
    case 1:
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0xFF);
        break;
    case 2:
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x00);
        break;
    case 3:
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0x00);
        break;

    default:
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, ~GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1));
        break;
    }
}


/*
* Hardware dependencies:
* 	PE0 - AIN3 - Reads analog value from sensor(@1MSps/64)
* 		Hardware averaging of 64 samples
* 	PF1 - PWMOut1 - Controls scanner angle (PWM block 0, Generator 0, Output 1)
* 		Gen0 generates ~60Hz PWM (31249 passed as argument)
* 		PWMOut1 runs between 26980 and 29870
* 	PF2/PF3 - PWMOut2/PWMOut3 - Control left/right engine PWM signal (PWM block 0, Generator 1, Outputs 2,3)
* 		Gen1 generates ~60Hz PWM (31249 passed as argument)	-could be increased to get more power????
* 		PWMOut2/PWMOut3 run between 5 and 31249
* 	PG0/PG1/PK4 -PWMOut4/PWMOut5/PWMOut6 - Control Node1/Node2/Rotation of robotic arm (PWM block 0, Generators 3,4, Outputs 4,5,6)
* 		Gen2 generates ~60Hz PWM (31249 passed as argument)
* 		PWM out4/PWMOut5/PWMOut6 runs between 26980 and 29870
* 	PL0/PL1/PL2/PL3 - GPIO - H-bridge configuration for engines
* 		PL1-PL0 -> sets direction of left engine (see direction Macros)
* 		PL3-PL2 -> sets direction of right engine (see direction Macros)
* 	PP0/PP1 - GPIO - Optical encoders for each wheel, resultion of 90 slits per rotation
* 		Interrupt based
* 		PP0 -> counts left engine
* 		PP1 -> counts right engine
* 	SPI2 - Used to communicate with WiFi module
* 		PB2 - Low ->WiFi module able to receive data over SPI
* 		PB3 - WiFi module has data to be sent to TM4C
* 		PD7 - CS pin -> slave select
* 		PD3 - SCLK -> SPI clock line
* 		PD1 - SPI Rx
* 		PD0 - SPI Tx
 */


int main(void)
{
    HAL_BOARD_CLOCK_Init();
    UartHW comm;
    TaskScheduler ts;

    comm.InitHW();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0x00);

    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0|GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0|GPIO_PIN_1,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOPinWrite(GPIO_PORTJ_BASE,GPIO_PIN_0|GPIO_PIN_1,0x00);

comm.Send("Board initialized!\r\n");
    //rpiRov.InitHW();

/************Dummy kernel module*********/
    volatile struct _callBackEntry LEDmodule;
    LEDmodule.args = taskArgs;
    LEDmodule.callBackFunc = LEDCallback;
    LEDmodule.serviceID = &taskID;
    TS_RegCallback(LEDmodule, 0);
/************Dummy kernel module*********/


    ts.PushBackEntrySync(0, 0, 1000);
    ts.PushBackEntrySync(0, 1, 2000);
    ts.PushBackEntrySync(0, 2, 3000);
    ts.PushBackEntrySync(0, 3, 3000);
    ts.PushBackEntrySync(0, 4, 5000);
    ts.PushBackEntrySync(0, 5, 7000);

    while(1)
    {

    	//rpiRov.RPICallback();
        //rpiRov.Resp();
    	//UARTprintf("LEFT: %d  RIGHT: %d  \n", __pED->wheelCounter[ED_LEFT], __pED->wheelCounter[ED_RIGHT]);
    	//SysCtlDelay(g_ui32SysClock/5);
    }

}
