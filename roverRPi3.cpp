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
    //comm.Send("Recvd(%d):  %s\n", sockID, buf);

    __taskSch->SyncTask(ESP_UID,ESP_T_SENDTCP,-1500);
    __taskSch->AddArgs(&sockID, 1);
    __taskSch->AddArgs(buf, *len);
}

int main(void)
{
    HAL_BOARD_CLOCK_Init();
    volatile TaskScheduler ts;

    //  Initialize UART port for connection with PC (for debugging)
    comm.InitHW();

    comm.Send("Initializing ESP\n");
    //  Initialize hardware used to talk to ESP8266
    esp.InitHW();
    //  Add hook for processing received data from ESP's sockets
    esp.AddHook(RxHook);
    comm.Send("Connecting to AP\n");
    //  Connect to AP
    esp.ConnectAP("sgvfyj7a", "7vxy3b5d");

    comm.Send("Board initialized!\r\n");

    uint16_t tmp;

    /*
     * Continuously connect to TCP server at +1s after scheduling, attempt to
     * send 25bytes of data at +2s after scheduling and then close TCP socket
     * at +10s after scheduling. Hook for processing Rx data will echo the data
     * to the same socket it came from effectively retransmitting the message
     * until the socket is closed at T+10s.
     */
    while (1)
    {
        ts.SyncTask(ESP_UID,ESP_T_CONNTCP,-1000);
        tmp = 1;
        ts.AddArgs(&tmp, 1);
        ts.AddArgs((void*)"192.168.0.11", 12);
        tmp = 2701;
        ts.AddArgs(&tmp, 2);

        ts.SyncTask(ESP_UID,ESP_T_SENDTCP,-2000);
        tmp = 0;
        ts.AddArgs(&tmp, 1);
        ts.AddArgs((void*)"Hello from ESP module!!\r\n", 25);

        ts.SyncTask(ESP_UID, ESP_T_CLOSETCP, -10000);
        ts.AddArgs(&tmp, 1);

        //  Wait until all tasks are processed
        while(!ts.IsEmpty())
        {
            TS_GlobalCheck();
        }
    }
}
