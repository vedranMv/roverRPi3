#include "init/platform.h"
#include "HAL/hal.h"

#ifdef __FAULT_STARTUP_DEBUG__
#include "serialPort/uartHW.h"
#endif

//  Rover platform
Platform& rover = Platform::GetI();

int main(void)
{
    //  Initialize board and FPU
    HAL_BOARD_CLOCK_Init();


#ifdef __FAULT_STARTUP_DEBUG__
    //  Initialize UART connection with PC (for debugging)
    SerialPort::GetI().InitHW();
    DEBUG_WRITE("Debug port initialized \n");
#endif
    //  Run initialization sequence for all modules of rover
    rover.InitHW();
#ifdef __FAULT_STARTUP_DEBUG__
    DEBUG_WRITE("Board initialized!\r\n");
#endif
    while(1)
        TS_GlobalCheck();

}
