/**
 * platform.cpp
 *
 *  Created on: Mar 10, 2017
 *      Author: Vedran
 */
#include <roverKernel/init/hooks.h>
#include <roverKernel/init/platform.h>
#include "roverKernel/libs/myLib.h"

#include "utils/uartstdio.h"

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
//        esp->ConnectAP("sgvfyj7a", "7vxy3b5d");
//        //  Initialize data streams and bind them to sockets
//        DataStream_InitHW();
//        telemetry.BindToSocketID(P_TO_SOCK(P_TELEMETRY));
//        commands.BindToSocketID(P_TO_SOCK(P_COMMANDS));

#endif

        //  Run post-initialization stuff
        _PostInit();
}

/**
 * Decode receiving message
 * Message frame:
 * Sender:timestamp(int32_t):repeats(int8_t):argLen(2B)::libUID(1B)serviceID(1B):args\r\n
 * @param buf
 * @param len
 */
void Platform::DecodeIncoming(const uint8_t* buf, const uint16_t len)
{
    uint16_t it = 2, argLen = 0;
    uint8_t libUID, serviceID;
    int32_t timestamp = 0;

    //  Find first : ->after it starts time stamp for scheduling task; when
    //  finished, iterator is at first char of time stamp
    while( buf[it++] != ':');
    //  Now go until next : to find length of time stamp string; when finished,
    //  iterator is at first char of repeats
    while( buf[it++] != ':')
        argLen++;
    //  Convert string to int
    timestamp = stoi((uint8_t*)(buf+it-argLen-1), argLen);
    SerialPort::GetI().Send("Task pending at %d \n", timestamp);

    //  Find :: ->that's beginning of the message; when done, iterator is
    //  at first char of message (also first char of libUID)
    while ( !((buf[it-2] == 0x3A) && (buf[it-1] == 0x3A)) )
        it++;
    //  Extract length of arguments
    memcpy(&argLen, (char*)(buf+it-4) , 2);
    //  Extract kernel module id and move iterator to service id
    memcpy(&libUID, (char*)(buf+it), 1);
    it++;
    //  Extract service id and move iterator to first byte of arguments
    memcpy(&serviceID, (char*)(buf+it), 1);
    it+=2;

    //  Schedule non-periodic task ASAP
    ts->SyncTask(libUID, serviceID, timestamp);

    //  Pass location and size of arguments
    ts->AddArgs((void*)(buf+it), argLen);

    //TODO: Do something useful with timestamp
}

void Platform::_PostInit()
{
#ifdef __HAL_USE_MPU9250__
    mpu->Listen(true);
#endif

}

///-----------------------------------------------------------------------------
///                      Class constructor & destructor              [PROTECTED]
///-----------------------------------------------------------------------------
Platform::Platform()
    : telemetry(TCP_SERVER_IP, P_TELEMETRY), commands(TCP_SERVER_IP, P_COMMANDS)
{}
Platform::~Platform()
{}
