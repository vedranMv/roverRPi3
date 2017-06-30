#include "init/platform.h"
#include "libs/myLib.h"
#include "serialPort/uartHW.h"
#include "HAL/hal.h"


//  Debug bridge UART<->USB
SerialPort &comm2 = SerialPort::GetI();
//  Rover platform
Platform& rover = Platform::GetI();

/**
 * Small kernel module for scheduling periodic print-out of data
 */
struct _kernelEntry dataDump;
void dataDumpCallback(void)
{
    EngineData& eng = EngineData::GetI();
    comm2.Send("%6d::: LEFT: %d   RIGHT: %d  \n", msSinceStartup, eng.wheelCounter[0], eng.wheelCounter[1]);
}

int main(void)
{

    HAL_BOARD_CLOCK_Init();

    //  Initialize UART port for connection with PC (for debugging)
    comm2.InitHW();
    comm2.Send("Debug port initialized \n");

    //  Register data-dump kernel module
    dataDump.callBackFunc = dataDumpCallback;
    TS_RegCallback(&dataDump, 7);

    rover.InitHW();
    comm2.Send("Board initialized!\r\n");

    //  Schedule radar scan at T+2s
    //rover.ts->SyncTask(RADAR_UID, RADAR_SCAN, -2000);
    //rover.ts->AddArg<uint8_t>(0);   //  Schedule coarse scan, 160 points

    //  Schedule periodic printing of encoder readouts
    //  -1 repeats means task is repeated indefinitely
//  rover.ts->SyncTask(7, 0, 1000, true, -1);

    //  Schedule periodic dump of sensor data 2 times per second
    rover.ts->SyncTask(MPU_UID, MPU_GET_DATA, 500, true, -1);
    rover.ts->AddArg<uint8_t>(0);

    while(1)
        TS_GlobalCheck();
}

/*

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

    //  Connect to TCP server at T+1s
    //  ------MAKE SURE IP ADDRESS IS CORRECT-------------------
    rover.ts->SyncTask(ESP_UID,ESP_T_CONNTCP,-1000);
    rover.ts->AddArg<uint8_t>(1);                  //  Keep alive
    rover.ts->AddArgs((void*)"192.168.0.16", 12);  //  IP address
    rover.ts->AddArg<uint16_t>(2700);              //  TCP port
    rover.ts->AddArg<uint8_t>(0);
    //  Close TCP socket T+20s
    rover.ts->SyncTask(ESP_UID,ESP_T_CLOSETCP,-40000);
    rover.ts->AddArg<uint8_t>(0);   //  Close socket ID=0

    //
    // Schedule two periodic task with the same period but shifted for 1/2 phase
    // 1st task drives forward for 5 cm every 4s
    // 2nd task drives backwards for 5 cm every 4s
    //--------------------------------------------------------------------------
    //
    rover.ts->SyncTask(ENGINES_UID, ENG_T_MOVE_ENG, 6000, true, 5);
    rover.ts->AddArg<uint8_t>(ENG_DIR_FW);
    rover.ts->AddArg<float>(10.00);
    rover.ts->AddArg<uint8_t>(0);

    HAL_DelayUS(3000000);

    rover.ts->SyncTask(ENGINES_UID, ENG_T_MOVE_ENG, 6000, true, 5);
    rover.ts->AddArg<uint8_t>(ENG_DIR_BW);
    rover.ts->AddArg<float>(10.00);
    rover.ts->AddArg<uint8_t>(0);

    //  Buttons and LEDs initialization
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
