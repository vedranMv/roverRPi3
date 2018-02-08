/**
 * platform.cpp
 *
 *  Created on: Mar 10, 2017
 *      Author: Vedran
 */
#include "init/platform.h"
#include "init/hooks.h"
#include "libs/myLib.h"
#include "HAL/hal.h"
#include "init/eventLog.h"

#include <string>
#include <sstream>

//  Enable debug information printed on serial port
//#define __DEBUG_SESSION__

#ifdef __DEBUG_SESSION__
#include "serialPort/uartHW.h"

#endif

//  Integration with event log, if it's present
#ifdef __HAL_USE_EVENTLOG__
    #include "init/eventLog.h"
    //  Simplify emitting events
    #define EMIT_EV(X, Y)  EventLog::EmitEvent(PLAT_UID, X, Y)
#endif /* __HAL_USE_EVENTLOG__ */

/**
 * Template function to convert any number into a std::string
 * @param t Number of any type
 * @return Number passed in argument t as std::string
 */
template <typename T> inline std::string tostr(const T& t) {
   std::ostringstream os;
   os<<t;
   return os.str();
}

/**
 * Callback routine to invoke service offered by this module from task scheduler
 * @note It is assumed that once this function is called task scheduler has
 * already copied required variables into the memory space provided for it.
 */
void _PLAT_KernelCallback(void)
{
    Platform  &__plat = Platform::GetI();

    //  Check for null-pointer
    if (__plat._platKer.args == 0)
        return;
    //__plat.mpu.Listen(false);
    /*
     *  Data in args[] contains bytes that constitute arguments for function
     *  calls. The exact representation(i.e. whether bytes represent ints, floats)
     *  of data is known only to individual blocks of switch() function. There
     *  is no predefined data separator between arguments inside args[].
     */
    switch (__plat._platKer.serviceID)
    {
    /*
     *  Pack & send telemetry frame
     *  args[] = none
     *  retVal on of myLib.h STATUS_* macros
     */
    case PLAT_T_TEL:
        {
            /*
             * Telemetry frame has the following format:
             * @note numbers are represented as strings not byte values
             * timeSinceStartup:Roll:Pitch:Yaw:distanceLeft:distanceRight:speedLeft:speedRight:accX:accY:accZ\n
             */
            std::string telemetryFrame;
            float rpy[3];

            //  Starting sequence "1*" marks beginning of standard telemetry
            //  frame with all sensor data
            telemetryFrame = "1*:" + tostr(msSinceStartup) + ":";
#ifdef __HAL_USE_MPU9250__
            //  Get RPY orientation on degrees
            __plat.mpu->RPY(rpy, true);
            telemetryFrame += tostr(rpy[0])+":"+tostr(rpy[1])+":"+tostr(rpy[2])+":";
#else
            telemetryFrame += tostr<float>(0.0)+":"+tostr<float>(0.0)+":"+tostr<float>(0.0)+":";
#endif

            //  Get 3-axis acceleration from MPU
            float acc[3];
            __plat.mpu->Acceleration(acc);

            //  Write engine telemetry into the packet
            telemetryFrame += tostr<float>((float)__plat.eng->GetDistance(0)) + ":";
            telemetryFrame += tostr<float>((float)__plat.eng->GetDistance(1)) + ":";
            telemetryFrame += tostr<float>((float)__plat.eng->wheelSpeed[0]) + ":";
            telemetryFrame += tostr<float>((float)__plat.eng->wheelSpeed[1]) + ":";
            telemetryFrame += tostr<float>(acc[0]) + ":";
            telemetryFrame += tostr<float>(acc[1]) + ":";
            telemetryFrame += tostr<float>(acc[2]) + ":";

            telemetryFrame += '\n';

            //  Send over telemetry stream
            __plat._platKer.retVal =
                    __plat.telemetry.Send((uint8_t*)telemetryFrame.c_str());

#ifdef __DEBUG_SESSION__
            DEBUG_WRITE("\nSending frame(%d), len:%d \n  %s \n",     \
                    __plat._platKer.retVal, telemetryFrame.length(),   \
                    telemetryFrame.c_str());
#endif

            //  If previous sending failed, no need to force next sending, pass
            if (__plat._platKer.retVal != STATUS_OK)
                return;

            //  If there are any unsent events, ship them off now
            if (EventLog::GetI().EventCount() > 0)
            {
                //  Loop through linked list of events and send them one by one
                volatile struct _eventEntry* node = EventLog::GetI().GetHead();
                uint16_t nodesSent = 1;
                while(node != 0)
                {
                    //  Assemble telemetry frame from event log
                    //  Starting sequence "2*" marks beginning of frame carrying
                    //  event log data, one log entry per frame
                    telemetryFrame =  "2*:" + tostr<uint16_t>(EventLog::GetI().EventCount()-nodesSent) + ":";
                    telemetryFrame += "[" + tostr<uint32_t>(node->timestamp) + "]:";
                    telemetryFrame += tostr<uint16_t>(node->libUID) + ":";
                    telemetryFrame += tostr<int16_t>(node->taskID) + ":";
                    telemetryFrame += tostr<uint16_t>(node->event) + ":";

                    __plat.telemetry.Send((uint8_t*)telemetryFrame.c_str(),
                                                   telemetryFrame.length());

#ifdef __DEBUG_SESSION__
                    DEBUG_WRITE("\nSending frame(%d), len:%d \n  %s \n",     \
                            __plat._platKer.retVal, telemetryFrame.length(),   \
                            telemetryFrame.c_str());
#endif

                    node = node->next;
                    nodesSent++;
                }
            }

            //  Telemetry doesn't affect status, if it fails, software
            //  does best-effort to try and resend it
            __plat._platKer.retVal = STATUS_OK;
            return; //  No need to report status, telemetry is not important
        }
    /*
     * Reboot microcontroller
     * args[] = rebootCode(0x17)
     * retVal none
     */
    case PLAT_T_REBOOT:
        {
            //  Reboot only if 0x17 was sent as argument
            if (__plat._platKer.args[0] == 0x17)
            {
                HAL_BOARD_Reset();
            }
        }
        break;
    /*
     * Send only highest priority events emitted until now
     * args[] = none
     * retVal STATUS_OK
     */
    case PLAT_T_EVLOG_DUMP:
        {
            std::string telemetryFrame;

            for (uint8_t i = 0; i < NUM_OF_MODULES; i++)
            {
                struct _eventEntry ee = EventLog::GetI().GetHigPrioEvAt(i);

                //  (-1) is default initialization value, means there's no entry
                //  yet for this module in event log
                if (ee.libUID == (-1))
                    continue;

                //  Construct standard telemetry frame with event log data, format:
                //  2*:numOfEvents:[time]:libUID:taskUID:event
                //  NOTE: First argument numOfEvents is here set to 5, it can be
                //  any number !=0. When 0 is sent client will request DropBefore(time)
                //  function event log, deleting all entries before given time
                telemetryFrame =  "2*:" + tostr<uint16_t>(5) + ":";
                telemetryFrame += "[" + tostr<uint32_t>(ee.timestamp) + "]:";
                telemetryFrame += tostr<uint16_t>(ee.libUID) + ":";
                telemetryFrame += tostr<int16_t>(ee.taskID) + ":";
                telemetryFrame += tostr<uint16_t>(ee.event) + ":";

                //  Send telemetry frame
                __plat.telemetry.Send((uint8_t*)telemetryFrame.c_str(),
                                               telemetryFrame.length());
            }
            //  Telemetry can't affect status, it's only a best-effort to
            //  deliver data
            __plat._platKer.retVal = STATUS_OK;
        }
        break;
    /*
     * Perform soft reset of platform module, reset only event log state
     * args[] = rebootCode(0x17)
     * retVal STATUS_OK
     */
    case PLAT_T_SOFT_REBOOT:
        {
            //  Reboot only if 0x17 was sent as argument
            if (__plat._platKer.args[0] == 0x17)
            {
#ifdef __HAL_USE_EVENTLOG__
                EventLog::SoftReboot(PLAT_UID);
#endif  /* __HAL_USE_EVENTLOG__ */

            }
            //  There's nothing that can affect outcome of this task
            __plat._platKer.retVal = STATUS_OK;
        }
        break;
    /*
     * Send data about task scheduler performance and load
     * args[] = none
     * retVal STATUS_OK
     */
    case PLAT_T_TS_DUMP:
        {
            std::string telemetryFrame;
            uint32_t Ntasks = __plat.ts->NumOfTasks();

            for (uint8_t i = 0; i < Ntasks; i++)
            {
                const TaskEntry *task = __plat.ts->FetchNextTask(i==0);
                if (task == 0)
                    break;

                //  Construct standard telemetry frame with event log data, format:
                //  3*:[time]:pendingTasks
                telemetryFrame =  "3*:";
                telemetryFrame += "[" + tostr<uint32_t>((uint32_t)task->_timestamp) + "]:";
                telemetryFrame += tostr<uint16_t>(task->_libuid) + ":";
                telemetryFrame += tostr<uint16_t>(task->_task) + ":";
                telemetryFrame += tostr<int32_t>(task->_period) + ":";
                telemetryFrame += tostr<uint16_t>((uint16_t)task->_PID) + ":";

                //  Task performance data
                telemetryFrame += tostr<uint32_t>((uint32_t)task->_perf.taskRuns) + ":";
                telemetryFrame += tostr<uint32_t>((uint32_t)task->_perf.startTimeMissCnt) + ":";
                telemetryFrame += tostr<uint32_t>((uint32_t)task->_perf.startTimeMissTot) + ":";
                telemetryFrame += tostr<uint16_t>((uint16_t)task->_perf.msAcc) + ":";
                telemetryFrame += tostr<uint32_t>((uint32_t)task->_perf.accRT) + ":";
                telemetryFrame += tostr<uint16_t>((uint16_t)task->_perf.maxRT) + ":";

                //  Send telemetry frame
                __plat.telemetry.Send((uint8_t*)telemetryFrame.c_str(),
                                               telemetryFrame.length());
            }
            //  Telemetry can't affect status, it's only a best-effort to
            //  deliver data
            __plat._platKer.retVal = STATUS_OK;
        }
        break;
    /*
     * Send information about engines, current speed, distance traveled
     * args[] = none
     * retVal STATUS_OK
     */
    case PLAT_T_ENG_DUMP:
        {
            std::string telemetryFrame;
            float acc[3];

            __plat.mpu->Acceleration(acc);

            //Format:
            //  4*:distanceLeft:distanceRight:speedLeft:speedRight:accX:accY:accZ
            telemetryFrame =  "4*:";
            telemetryFrame += tostr<int32_t>((int32_t)__plat.eng->wheelCounter[0]) + ":";
            telemetryFrame += tostr<int32_t>((int32_t)__plat.eng->wheelCounter[1]) + ":";
            telemetryFrame += tostr<int32_t>((float)__plat.eng->wheelSpeed[0]) + ":";
            telemetryFrame += tostr<int32_t>((float)__plat.eng->wheelSpeed[1]) + ":";
            telemetryFrame += tostr<float>(acc[0]) + ":";
            telemetryFrame += tostr<float>(acc[1]) + ":";
            telemetryFrame += tostr<float>(acc[2]) + ":";


            #ifdef __DEBUG_SESSION__
                                DEBUG_WRITE("\nSending frame(%d), len:%d \n  %s \n",     \
                                        __plat._platKer.retVal, telemetryFrame.length(),   \
                                        telemetryFrame.c_str());
            #endif

            //  Send telemetry frame
            __plat.telemetry.Send((uint8_t*)telemetryFrame.c_str(),
                                           telemetryFrame.length());

            //  Telemetry can't affect status, it's only a best-effort to
            //  deliver data
            __plat._platKer.retVal = STATUS_OK;
        }
        break;
    default:
        break;
    }

    //  Check return-value and emit event based on it
#ifdef __HAL_USE_EVENTLOG__
    if (__plat._platKer.retVal == STATUS_OK)
        EMIT_EV(__plat._platKer.serviceID, EVENT_OK);
    else
        EMIT_EV(__plat._platKer.serviceID, EVENT_ERROR);
#endif  /* __HAL_USE_EVENTLOG__ */

}

///-----------------------------------------------------------------------------
///         Functions for returning static instance                     [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Return reference to a singleton
 * @return reference to an internal static instance
 */
Platform& Platform::GetI()
{
    static Platform singletonInstance;
    return singletonInstance;
}

/**
 * Return pointer to a singleton
 * @return pointer to a internal static instance
 */
Platform* Platform::GetP()
{
    return &(Platform::GetI());
}

///-----------------------------------------------------------------------------
///                      Class member function definitions              [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Initialize hardware for all modules being in use
 */
void Platform::InitHW()
{

    //  Run initialization of event logger
#ifdef __HAL_USE_EVENTLOG__
    EventLog::GetI().InitSW();
    EventLog::GetI().RecordEvents(true);
#endif  /* __HAL_USE_EVENTLOG__ */

    //  If using task scheduler get handle and start systick every 1ms
#ifdef __HAL_USE_TASKSCH__
        ts = TaskScheduler::GetP();
        ts->InitHW(1);
#endif

    //  Emit status of platform
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_STARTUP);
#endif  /* __HAL_USE_EVENTLOG__ */

    //  Register module services with task scheduler
    _platKer.callBackFunc = _PLAT_KernelCallback;
    TS_RegCallback(&_platKer, PLAT_UID);

    //  If using ESP chip, get handle and connect to access point
#ifdef __HAL_USE_ESP8266__
        esp = ESP8266::GetP();
        esp->InitHW();
        esp->AddHook(ESPDataReceived);
        //  Connect to AP in non-blocking mode, allowing everything else to
        //  be initialized while ESP establishes connection in the background
        //  and reports process through interrupt
        esp->ConnectAP("sgvfyj7a", "7vxy3b5d", true);
        //  Initialize data streams and bind them to sockets -> Since ESP is
        //  still connecting to AP they will gracefully fail to bind until
        //  connection is established (error handled by DataStream module)
        DataStream_InitHW();
        telemetry.BindToSocketID(P_TO_SOCK(P_TELEMETRY), true);

        //  Delay binding second socket so that the two tasks have different
        //  starting times, otherwise scheduler will be asked to execute them
        //  Concurrently which ends up in one of the tasks missing its start time
        HAL_DelayUS(20000); //  20ms delay
        commands.BindToSocketID(P_TO_SOCK(P_COMMANDS), true);
#endif
#ifdef __HAL_USE_ENGINES__
        eng = EngineData::GetP();
        eng->SetVehSpec(7.0f, 14.3f, 25.0f, 40);
        eng->InitHW();
#endif
#ifdef __HAL_USE_MPU9250__
        mpu = MPU9250::GetP();
        mpu->InitHW();
        mpu->InitSW();
#endif
#ifdef __HAL_USE_RADAR__
        rad = RadarModule::GetP();
        rad->InitHW();
        rad->AddHook(RADScanComplete);
#endif

        //  Run post-initialization stuff
        _PostInit();
}

/**
 * Decode received message which carries a task to be scheduled
 * Function extracts task and its arguments from message and schedules task
 * within task scheduler.
 * Message frame:
 * sender:libUID:serviceID:timestamp:repeats:argLen::args\r\n
 * (all parts of message except for 'args' are numbers represented as strings,
 * args value is encoded into bit field and needs can be memcpy-ed into variable)
 * @param buf
 * @param len
 */
void Platform::Execute(const uint8_t* buf, const uint16_t len, int *err)
{
    TaskEntry te;
    int32_t argv[10] = {0},
            argc = 0;

    //  Set error to 0 -> No error in parsing
    *err = STATUS_OK;

#ifdef __DEBUG_SESSION__
    DEBUG_WRITE("Received message(%d):\n  |>%s \n", len, buf);
#endif

    //  Count number of colons in message, if it's less than 8 message is corrupted
    uint8_t cnt = 0;
    for (uint16_t i = 0; i < len; i++)
        if (buf[i] == ':')
            cnt++;
    if (cnt < 8)
    {
        *err = STATUS_ARG_ERR;
#ifdef __DEBUG_SESSION__
    DEBUG_WRITE("Message is not in the correct format(%d)\n", cnt);
#endif
        return;
    }

    //  Iterator to keep track of position in string
    uint16_t it = 0;

    //  Skip first part of message which is the string identifying the sender
    while (buf[it++] != ':');

    //  Go through string and extract all arguments for task scheduling
    //  When finished 'it' points to first position of args
    while((it < (len-1)) && !((buf[it] == ':') && (buf[it+1] == ':')))
    {
        //  Temporary variables in which string is saved
        char tmp[10] = {0};
        uint8_t tmpLen = 0;

        //  Move from ':' to the first char after it
        while ((buf[it] == ':') && (it < len))
            it++;

        //  Extract all digits of a number
        while ((buf[it] != ':') && (it < len))
                tmp[tmpLen++] = buf[it++];

        //  Convert string to int and save it
        argv[argc++] = stoi((uint8_t*)tmp, tmpLen);
    }
    //  Skip double colon marking beginning of arguments
    it+=2;


#ifdef __DEBUG_SESSION__
    DEBUG_WRITE("Task has %d arguments, requests service %d from library %d \nArguments: ", argv[4], argv[1], argv[0]);
    for (int i = it; i < len; i++)
        DEBUG_WRITE("0x%X ", buf[i]);
    DEBUG_WRITE("\n");
#endif

    //  This one is special: Radar scan task needs to be repeated 160 times,
    //  with period of 40ms(to reposition radar head)
    if ((argv[0] == RADAR_UID) && (argv[1] == RADAR_T_SCAN))
    {
        argv[3] = 40;
        argv[4] = 160;
    }
    //  Schedule task based on data provided
    ts->SyncTaskPer(argv[0], argv[1], argv[2], argv[3], argv[4]);
    //  Pass location and size of arguments
    ts->AddArgs((void*)(buf+it), argv[5]);
}

/**
 * Post-initialization
 * Function runs (and schedules) all post-initialization tasks on the platform
 */
void Platform::_PostInit()
{
#ifdef __HAL_USE_MPU9250__
    //  Create periodic task that will read sensor data
    ts->SyncTaskPer(MPU_UID, MPU_T_GET_DATA, -50, 10, T_PERIODIC);
#endif

    //  Schedule periodic telemetry sending every 1s
    ts->SyncTaskPer(PLAT_UID, PLAT_T_TEL, -1000, 1000, T_PERIODIC);
    //  Startup speed loop for the engines
    ts->SyncTaskPer(ENGINES_UID, ENG_T_SPEEDLOOP, -150, 150, T_PERIODIC);

#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_OK);
#endif  /* __HAL_USE_EVENTLOG__ */

}

///-----------------------------------------------------------------------------
///                      Class constructor & destructor              [PROTECTED]
///-----------------------------------------------------------------------------
Platform::Platform()
    : telemetry(TCP_SERVER_IP, P_TELEMETRY), commands(TCP_SERVER_IP, P_COMMANDS)
{
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_UNINITIALIZED);
#endif  /* __HAL_USE_EVENTLOG__ */
}
Platform::~Platform()
{}
