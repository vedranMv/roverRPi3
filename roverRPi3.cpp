#include "init/platform.h"
#include "libs/myLib.h"
#include "serialPort/uartHW.h"
#include "HAL/hal.h"


//  Rover platform
Platform& rover = Platform::GetI();

int main(void)
{
    //  Initialize board and FPU
    HAL_BOARD_CLOCK_Init();

    //  Initialize UART connection with PC (for debugging)
    SerialPort::GetI().InitHW();
    DEBUG_WRITE("Debug port initialized \n");

    //  Run initialization sequence for all modules of rover
    rover.InitHW();
    DEBUG_WRITE("Board initialized!\r\n");

    while(1)
        TS_GlobalCheck();

}
