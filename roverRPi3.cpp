#include <roverKernel/init/platform.h>
#include "roverKernel/libs/myLib.h"
#include "roverKernel/serialPort/uartHW.h"
#include "roverKernel/HAL/hal.h"

//  Debug bridge UART<->USB
SerialPort&comm = SerialPort::GetI();
//  Rover platform
Platform& rover = Platform::GetI();

int main(void)
{

    HAL_BOARD_CLOCK_Init();

    //  Initialize UART port for connection with PC (for debugging)
    comm.InitHW();
    comm.Send("Debug port initialized \n");

    rover.InitHW();
    comm.Send("Board initialized!\r\n");

    //  Connect to TCP server at T+1s
    //  ------MAKE SURE IP ADDRESS IS CORRECT-------------------
    rover.ts->SyncTask(ESP_UID,ESP_T_CONNTCP,-1000);
    rover.ts->AddArg<uint8_t>(1);                  //  Keep alive
    rover.ts->AddArgs((void*)"192.168.0.16", 12);  //  IP address
    rover.ts->AddArg<uint16_t>(2701);              //  TCP port
    rover.ts->AddArg<uint8_t>(1);

    //  Schedule radar scan at T+2s
    rover.ts->SyncTask(RADAR_UID, RADAR_SCAN, -2000);
    rover.ts->AddArg<uint8_t>(0);   //  Schedule coarse scan, 160 points

    //  Close TCP socket T+20s
    rover.ts->SyncTask(ESP_UID,ESP_T_CLOSETCP,-40000);
    rover.ts->AddArg<uint8_t>(0);   //  Close socket ID=0


    while(1)
        TS_GlobalCheck();
}

/*


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

 */
