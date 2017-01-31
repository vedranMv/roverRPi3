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


void RxHook(uint8_t sockID, uint8_t *buf, uint16_t *len)
{
    comm.Send("Recvd(%d):  %s\n", sockID, buf);

    __taskSch->PushBack(ESP_UID,ESP_T_SENDTCP,-500);
    __taskSch->AddStringArg(&sockID, 1);
    __taskSch->AddStringArg(buf, *len);
}

int main(void)
{
    HAL_BOARD_CLOCK_Init();
    TaskScheduler ts;
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

    comm.Send("Board initialized!\r\n");

    uint16_t tmp;

    ts.PushBack(ESP_UID,ESP_T_CONNTCP,-1000);
    tmp = 1;
    ts.AddStringArg(&tmp, 1);
    ts.AddStringArg((void*)"192.168.0.11", 12);
    tmp = 2701;
    ts.AddStringArg(&tmp, 2);

    ts.PushBack(ESP_UID,ESP_T_SENDTCP,-2000);
    tmp = 0;
    ts.AddStringArg(&tmp, 1);
    ts.AddStringArg((void*)"Hello from ESP module!!\0", 23);

    ts.PushBack(ESP_UID, ESP_T_CLOSETCP, -10000);
    ts.AddStringArg(&tmp, 1);

    while(1)
    {
        TS_GlobalCheck();
    }

}
