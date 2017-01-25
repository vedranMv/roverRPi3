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

//#include "roverKernel/rpiDriver.h"
#include "roverKernel/tm4c1294_hal.h"

#include "roverKernel/uartHW.h"
#include "roverKernel/taskScheduler.h"
#include "roverKernel/esp8266.h"

UartHW comm;
ESP8266 esp;

//#define __DEBUG_SESSION__

//RPIRover rpiRov(6.8f, 14.3f, 25.0f, 40);

void RxHook(uint8_t *buf, uint16_t *len)
{
    uint8_t msg[30]={0};

    comm.Send("Recvd:  %s\n", buf);

    if ((buf[0] == 'H') && (buf[1] == 'e'))
    {
        //snprintf((char*)msg, 15,"12");
        msg[0]=17;
        __taskSch->PushBackEntrySync(0, 0, 0);//0 time - run task ASAP
       // __taskSch->AddArgForCurrent(msg,2);
        __taskSch->AddStringArg(msg, 2);

        snprintf((char*)msg, 30,"Hello there oyee!");
        __taskSch->PushBackEntrySync(0, 0, 0);//0 time - run task ASAP
        //__taskSch->AddArgForCurrent(msg,12);
        __taskSch->AddStringArg(msg, 12);
    }

    ///TODO: Resgister a call to task scheduler to send request to user
}

int main(void)
{
    HAL_BOARD_CLOCK_Init();

    comm.InitHW();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0x00);

    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0|GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0|GPIO_PIN_1,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOPinWrite(GPIO_PORTJ_BASE,GPIO_PIN_0|GPIO_PIN_1,0x00);


    esp.InitHW();
    esp.AddHook(RxHook);
    esp.ConnectAP("sgvfyj7a", "7vxy3b5d");
    esp.StartTCPServer(27541);
    esp.TCPListen(true);

    comm.Send("Board initialized!\r\n");
    TaskScheduler ts;


    while(1)
    {
        TS_GlobalCheck();
    }

}
