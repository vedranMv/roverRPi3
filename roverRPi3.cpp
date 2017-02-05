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


#include "roverKernel/tm4c1294_hal.h"

#include "roverKernel/uartHW.h"
#include "roverKernel/taskScheduler.h"
#include "roverKernel/esp8266.h"
#include "roverKernel/radarGP2.h"

UartHW comm;
ESP8266 esp;
RadarModule rm;


//RPIRover rpiRov(6.8f, 14.3f, 25.0f, 40);

/**
 * Function to be called when a new data is received from TCP clients on ALL
 * opened sockets at ESP. Function is called through data scheduler if enabled,
 * otherwise called directly from ISR
 * @param sockID socket ID at which the reply arrived
 * @param buf buffer containing incoming data
 * @param len size of incoming data in [buf] buffer
 */
void RxHook(uint8_t sockID, uint8_t *buf, uint16_t *len)
{
    uint8_t tmp;

    comm.Send("%s", buf);

    __taskSch->SyncTask(RADAR_UID, RADAR_SCAN, -2000);
    tmp = 0;
    __taskSch->AddArgs((void*)&tmp, 1);
}

void ScanComplete(uint8_t* data, uint16_t* dataLen)
{
    uint8_t tmp = 0;

    //  Print out scan on the serial
    /*for (uint16_t i = 0; i < *dataLen; i++)
        comm.Send("%d ", data[i]);*/

    data[*dataLen] = '\0';


    //  Schedule sending the data ASAP
    __taskSch->SyncTask(ESP_UID, ESP_T_SENDTCP, 0);
    __taskSch->AddArgs(&tmp, 1);
    tmp = ((uint8_t)*dataLen);
    UARTprintf("Scan returned %d \n", tmp);
    __taskSch->AddArgs(data, tmp);
}


int main(void)
{
    HAL_BOARD_CLOCK_Init();


    uint16_t tmp;
    volatile TaskScheduler ts;

    //  Initialize UART port for connection with PC (for debugging)
    comm.InitHW();
    rm.InitHW();
    comm.Send("HW initialized \n");
    rm.AddHook(ScanComplete);


    comm.Send("Initializing ESP\n");
    //  Initialize hardware used to talk to ESP8266
    esp.InitHW();
    //  Add hook for processing received data from ESP's sockets
    esp.AddHook(RxHook);
    comm.Send("Connecting to AP\n");
    //  Connect to AP
    esp.ConnectAP("sgvfyj7a", "7vxy3b5d");

    comm.Send("Board initialized!\r\n");

    //  Connect to TCP server at T+1s
    ts.SyncTask(ESP_UID,ESP_T_CONNTCP,-1000);
    tmp = 1;
    ts.AddArgs(&tmp, 1);
    ts.AddArgs((void*)"192.168.0.11", 12);
    tmp = 2701;
    ts.AddArgs(&tmp, 2);

    //  Schedule radar scan at T+2s
    ts.SyncTask(RADAR_UID, RADAR_SCAN, -2000);
    tmp = 0;
    ts.AddArgs(&tmp, 1);

    //  Close TCP socket T+20s
    ts.SyncTask(ESP_UID,ESP_T_CLOSETCP,-40000);
    tmp = 0;
    ts.AddArgs(&tmp, 1);

    while (1)
    {
        TS_GlobalCheck();
    }
}
