#include "init/platform.h"
#include "libs/myLib.h"
#include "serialPort/uartHW.h"
#include "HAL/hal.h"


//  Rover platform
Platform& rover = Platform::GetI();

/**
 * Small kernel module for testing PID loop on rover motors
 * Implementation tested here before merging with engines.h/cpp
 */
#define ENG_PID_UID         9
#define ENG_PID_T_DUMP      0   //  Dump wheel counters
#define ENG_PID_T_LOOP      1   //  PID loop for both wheels
#define ENG_PID_T_CHCOEF    2   //  Tweak PID coefficients
#define ENG_PID_T_WREVERSE  3   //  Detect change of direction


struct _kernelEntry engPID;
void dataDumpCallback(void)
{
    static float Kp[] = {0.7, 0.7}, Ki[] = {0.5, 0.5}, Kd[] = {0.0, 0.0};
    EngineData &__ed = EngineData::GetI();


    /*
     *  Data in args[] contains bytes that constitute arguments for function
     *  calls. The exact representation(i.e. whether bytes represent ints, floats)
     *  of data is known only to individual blocks of switch() function. There
     *  is no predefined data separator between arguments inside args[].
     */
    switch (engPID.serviceID)
    {
    /*
     * args[] = none
     * retVal one of myLib.h STATUS_* error codes
     */
    case ENG_PID_T_DUMP:
        {
            static int32_t oldWC[2] = {0,0};

            if ((oldWC[0] != __ed.wheelCounter[0]) || (oldWC[1] != __ed.wheelCounter[1]))
                DEBUG_WRITE("%6d::: LEFT: %d/%d   RIGHT: %d/%d  \n", msSinceStartup, __ed.wheelCounter[0], __ed.wheelSetPoint[0], __ed.wheelCounter[1], __ed.wheelSetPoint[1]);

            oldWC[0] = __ed.wheelCounter[0];
            oldWC[1] = __ed.wheelCounter[1];
        }
        break;
    /*
     * args[] = none
     * retVal one of myLib.h STATUS_* error codes
     */
    case ENG_PID_T_LOOP:
        {
            //  Variables from last entry to this loop
            static int32_t RoldE = 0, LoldE = 0;
            static float LP = 0, RP = 0;
            static const float dTLoop = 0.1;

            /// PID loop for position control - LEFT
            //  Calculate error
            int32_t error = __ed.wheelSetPoint[0]-__ed.wheelCounter[0];
            uint8_t dir;

            if (error < 0)
                dir = ENG_DIR_FW;  //  Lower 2 bits are left eng. config.
            else
                dir = ENG_DIR_BW;  //  Lower 2 bits are left eng. config.
            HAL_ENG_SetHBridge(0, dir);

            //  New, corrective PWM value
            float corr;
            LP += ((float)(error))*dTLoop;  //  Add error to sum for integral component
            corr = Kp[0]*((float)(error)) + Ki[0]*LP;
            HAL_ENG_SetPWM(0, corr);

        }
        break;
    /*
     * args[] = Kp(float)|Ki(float)|Kd(float)
     * retVal one of myLib.h STATUS_* error codes
     */
    case ENG_PID_T_CHCOEF:
        {
            //  Check for null-pointer for data
            if (engPID.args == 0) return;
            //  Check for sufficient data length
            if (engPID.argN != 24) return;

            //  Copy data into internal coefficients
            memcpy((float*)(Kp), (void*)engPID.args, sizeof(float));
            memcpy((float*)(Ki), (void*)(engPID.args+1*sizeof(float)), sizeof(float));
            memcpy((float*)(Kd), (void*)(engPID.args+2*sizeof(float)), sizeof(float));
            memcpy((float*)(Kp+1), (void*)(engPID.args+3*sizeof(float)), sizeof(float));
            memcpy((float*)(Ki+1), (void*)(engPID.args+4*sizeof(float)), sizeof(float));
            memcpy((float*)(Kd+1), (void*)(engPID.args+5*sizeof(float)), sizeof(float));

        }
        break;
    /*
     * args[] = none
     * retVal one of myLib.h STATUS_* error codes
     */
    case ENG_PID_T_WREVERSE:
        {
            static int32_t oldWC[] = {0, 0};
            static float oldSpeed[] = {0.0, 0.0};
            const static float dT = 0.01;   //  Called every 10 ms

            float speedL = (float)(__ed.wheelCounter[0]-oldWC[0])/dT;

            oldWC[0] = __ed.wheelCounter[0];
            oldWC[1] = __ed.wheelCounter[1];
        }
        break;

    default:
        break;
    }

}


int main(void)
{

    HAL_BOARD_CLOCK_Init();

    //  Initialize UART connection with PC (for debugging)
    SerialPort::GetI().InitHW();
    DEBUG_WRITE("Debug port initialized \n");

    //  Register data-dump kernel module
    //engPID.callBackFunc = dataDumpCallback;
    //TS_RegCallback(&engPID, ENG_PID_UID);

    rover.InitHW();
    DEBUG_WRITE("Board initialized!\r\n");

    //  Schedule PID loop on the wheels every dT=100 ms
    //rover.ts->SyncTaskPer(ENG_PID_UID, ENG_PID_T_DUMP, -100, 100, -1);

    //  Schedule radar scan at T+2s
    //rover.ts->SyncTask(RADAR_UID, RADAR_SCAN, -2000);
    //rover.ts->AddArg<uint8_t>(0);   //  Schedule coarse scan, 160 points

    //  Schedule periodic printing of encoder readouts
    //  -1 repeats means task is repeated indefinitely
    //rover.ts->SyncTask(7, 0, 1000, true, -1);

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
