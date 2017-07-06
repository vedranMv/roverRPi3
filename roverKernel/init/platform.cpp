/**
 * platform.cpp
 *
 *  Created on: Mar 10, 2017
 *      Author: Vedran
 */
#include "init/hooks.h"
#include "init/platform.h"
#include "libs/myLib.h"
#include "HAL/hal.h"
#include "init/eventLog.h"

#include <string>
#include <sstream>

//#define __DEBUG_SESSION__

#ifdef __DEBUG_SESSION__
#include "serialPort/uartHW.h"

#endif

#define EMIT_EV(X, Y)  EventLog::EmitEvent(PLAT_UID, X, Y)


//  TODO: In test -> measure how much stack uses
template <typename T> std::string tostr(const T& t) {
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
    Platform* __plat = Platform::GetP();
    //  Check for null-pointer
    if (__plat->_platKer.args == 0)
        return;
    /*
     *  Data in args[] contains bytes that constitute arguments for function
     *  calls. The exact representation(i.e. whether bytes represent ints, floats)
     *  of data is known only to individual blocks of switch() function. There
     *  is no predefined data separator between arguments inside args[].
     */
    switch (__plat->_platKer.serviceID)
    {
    /*
     *  Pack & send telemetry frame
     *  args[] = none
     *  retVal on of myLib.h STATUS_* macros
     */
    case PLAT_TEL:
        {
            /*
             * Telemetry frame has the following format:
             * @note numbers are represented as strings not byte values
             * [timeSinceStartup]:leftEncoder|rightEncoder|Roll|Pitch|Yaw\0
             */
            std::string telemetryFrame;
            float rpy[3];

            //  Starting sequence "1*" marks beginning of standard telemetry
            //  frame with all sensor data
            telemetryFrame = "1*:[" + tostr(msSinceStartup) + "]:";
#ifdef __HAL_USE_MPU9250__
            //  Get RPY orientation on degrees
            __plat->mpu->RPY(rpy, true);
            telemetryFrame += tostr(rpy[0])+"|"+tostr(rpy[1])+"|"+tostr(rpy[2])+"|";
#endif
            telemetryFrame += '\n';
            EventLog::GetI().RecordEvents(true);
            //  Send over telemetry stream
            __plat->_platKer.retVal =
                    __plat->telemetry.Send((uint8_t*)telemetryFrame.c_str());

#ifdef __DEBUG_SESSION__
            DEBUG_WRITE("\nSending frame(%d):%d \n  %s \n",     \
                    __plat->_platKer.retVal, telemetryFrame.length(),   \
                    telemetryFrame.c_str());
#endif

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
                    telemetryFrame += tostr<uint16_t>(node->taskID) + ":";
                    telemetryFrame += tostr<uint16_t>(node->event) + ":";
                    //  Append new error code (use of OR) to not overwrite old one
                    __plat->_platKer.retVal |=
                            __plat->telemetry.Send((uint8_t*)telemetryFrame.c_str(),
                                                   telemetryFrame.length());

#ifdef __DEBUG_SESSION__
                    DEBUG_WRITE("\nSending frame(%d):\n  %s \n", telemetryFrame.length(), telemetryFrame.c_str());
#endif

                    node = node->next;
                    nodesSent++;
                }
            }
        }
        break;
    /*
     * Reboot microcontroller
     * args[] = rebootCode(1B)
     * retVal none
     */
    case PLAT_REBOOT:
        {
            //  Reboot only if 0x17 was sent as argument
            if (__plat->_platKer.args[0] == 0x17)
            {
                HAL_BOARD_Reset();
            }
        }
        break;
    default:
        break;
    }

    //  Check return-value and emit event based on it
#ifdef __HAL_USE_EVENTLOG__
    if (__plat->_platKer.retVal == STATUS_OK)
        EMIT_EV(__plat->_platKer.serviceID, EVENT_OK);
    else
        EMIT_EV(__plat->_platKer.serviceID, EVENT_ERROR);
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
    //  Emit status of platform
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_STARTUP);
#endif  /* __HAL_USE_EVENTLOG__ */

    //  Register module services with task scheduler
    _platKer.callBackFunc = _PLAT_KernelCallback;
    TS_RegCallback(&_platKer, PLAT_UID);

    //  If using task scheduler get handle and start systick every 1ms
#ifdef __HAL_USE_TASKSCH__
        ts = TaskScheduler::GetP();
        ts->InitHW(1);
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
//  If using ESP chip, get handle and connect to access point
#ifdef __HAL_USE_ESP8266__
        esp = ESP8266::GetP();
        esp->InitHW();
        esp->AddHook(ESPDataReceived);
        esp->ConnectAP("sgvfyj7a", "7vxy3b5d");
        //  Initialize data streams and bind them to sockets
        DataStream_InitHW();
        telemetry.BindToSocketID(P_TO_SOCK(P_TELEMETRY));
        commands.BindToSocketID(P_TO_SOCK(P_COMMANDS));
#endif

#ifdef __HAL_USE_EVENTLOG__
    EventLog::GetI().InitSW();
    EMIT_EV(-1, EVENT_INITIALIZED);
#endif  /* __HAL_USE_EVENTLOG__ */

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

    //  Count number of colons in message, if it's less than 7 message is corrupted
    uint8_t cnt = 0;
    for (uint16_t i = 0; i < len; i++)
        if (buf[i] == ':')
            cnt++;
    if (cnt < 7)
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
    DEBUG_WRITE("Task has %d arguments, requests service %d from \
            library %d \nArguments: ", argv[4], argv[1], argv[0]);
    for (int i = it; i < len; i++)
        DEBUG_WRITE("0x%X ", buf[i]);
    DEBUG_WRITE("\n");
#endif
    //  Schedule non-periodic task based on data provided
    ts->SyncTask(argv[0], argv[1], argv[2], (argv[3]!=0)?true:false, argv[3]);

    //  Pass location and size of arguments
    ts->AddArgs((void*)(buf+it), argv[4]);
}

/**
 * Post-initialization
 * Function executes all the post-initialization tasks on the platform
 */
void Platform::_PostInit()
{
#ifdef __HAL_USE_MPU9250__
    //  Start listening for sensor data
    mpu->Listen(true);
#endif

    //  Schedule periodic telemetry sending every 1.5s
    ts->SyncTask(PLAT_UID, PLAT_TEL, 1500, true, T_PERIODIC);

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
