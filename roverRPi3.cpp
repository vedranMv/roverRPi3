#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "roverKernel/myLib.h"

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

//  Kernel includes
#include "roverKernel/tm4c1294_hal.h"
#include "roverKernel/uartHW.h"
#include "roverKernel/taskScheduler.h"
#include "roverKernel/esp8266.h"
#include "roverKernel/radarGP2.h"
#include "roverKernel/engines.h"
#include "roverKernel/mpu9250.h"

//  Kernel modules under test
UartHW comm;
MPU9250 mpu;


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

    //  Print received data
    comm.Send("%s", buf);
    //  Schedule new radar scan at T+2s
    __taskSch->SyncTask(RADAR_UID, RADAR_SCAN, -2000);
    tmp = 0;
    __taskSch->AddArgs((void*)&tmp, 1);
}

void mpuHook(float *acc, float* gyro)
{

}


int main(void)
{
    HAL_BOARD_CLOCK_Init();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0x00);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0|GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE,
                    GPIO_PIN_0|GPIO_PIN_1,
                    GPIO_STRENGTH_8MA,
                    GPIO_PIN_TYPE_STD_WPU);
    GPIOPinWrite(GPIO_PORTJ_BASE,GPIO_PIN_0|GPIO_PIN_1,0x00);


    //  Initialize after setting clock, it uses board-clock for SysTick
    volatile TaskScheduler ts;

    //  Initialize UART port for connection with PC (for debugging)
    comm.InitHW();
    comm.Send("HW initialized \n");

    mpu.InitHW();
    mpu.InitSW();
    //mpu.AddHook(mpuHook);
    mpu.Listen(true);

    comm.Send("Mag: %d \n", HAL_MPU_ReadByte(AK8963_ADDRESS, WHO_AM_I_AK8963));

    while(1);

    while (GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == GPIO_PIN_0)
    {
        float roll, pitch, yaw;
        TS_GlobalCheck();
        if (mpu.IsDataReady())
        {
            mpu.IMU()->GetOrientation(&roll, &pitch, &yaw);
            UARTprintf("R: %d   P: %d   Y: %d \n", lroundf(roll),
                                                   lroundf(pitch),
                                                   lroundf(yaw));
        }
    }

    HAL_ENG_Enable(ED_BOTH, false);
    comm.Send("Done \n");
    while(1);
}

/*
ESP8266 esp;
RadarModule rm;
EngineData ed;
RadarModule rd;


    //  Initialize radar so it doesn't twitch when the rover is powered on
    rd.InitHW();

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

    ed.InitHW();
    comm.Send("Motors initialized \n");
    ed.SetVehSpec(7.0f, 14.3f, 25.0f, 40);
    com.Send("Specs set \n");
    //
    // Create periodic task, repeated 3 times, with period of 3s
    // On every task execution moves the vehicle 5cm forward, doesn't wait
    // for the vehicle to complete the action.
    // -------------------------------------------------------------------------
    //
    ts.SyncTask(ENGINES_UID, ENG_MOVE_ENG, -3000, true, 3);
    ts.AddArg((uint8_t)DIRECTION_FORWARD);
    ts.AddArg(5.0f);
    ts.AddArg(false);

 */
