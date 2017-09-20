/**
 * platform.h
 *
 *  Created on: Mar 10, 2017
 *      Author: Vedran Mikov
 *
 *  Platform contains high-level definition of all modules currently attached and
 *  needed on the platform. It is thought of as a single module to unify
 *  low-level drivers and provide single point for initialization of platform.
 */

#ifndef ROVERKERNEL_INIT_PLATFORM_H_
#define ROVERKERNEL_INIT_PLATFORM_H_

#include "hwconfig.h"
#include "engines/engines.h"
#include "esp8266/esp8266.h"
#include "radar/radarGP2.h"
#include "mpu9250/mpu9250.h"
#include "taskScheduler/taskScheduler.h"

#include "network/dataStream.h"


/**     TCP port definitions for standard data streams   */
/*
 * Macro for relating TCP port and socket handle as returned by ESP8266
 * SOCKET_ID = TCP_PORT - SOCKET_OFFSET
 */
#define P_TO_SOCK(X)     (uint8_t)(X-2700)
#define SOCK_TO_P(X)    ((uint16_t)X+2700)
/*
 * Telemetry data stream
 * Telemetry includes bidirectional stream starting from rover to server containing
 * sensor data, time reference, health report etc. On received frame from rover
 * server replies with "ACK\r\n"
 * Server expects telemetry stream on TCP port 2700
 */
#define P_TELEMETRY     2700
/*
 * Commands data stream
 * This stream brings commands from server to rover. On received frame from
 * server rover replies "ACK\r\n"
 * Server expects commands stream on TCP port 2701
 */
#define P_COMMANDS      2701

//#define TCP_SERVER_IP   (uint8_t*)"192.168.0.12\0"
#define TCP_SERVER_IP   (uint8_t*)"192.168.0.29\0"

/*
 * Kernel module UID and task IDs
 */
    //  Unique identifier of this module as registered in task scheduler
    #define PLAT_UID    5
    //  Definitions of ServiceID for service offered by this module
    #define PLAT_T_TEL            0   //  Send telemetry data frame
    #define PLAT_T_REBOOT         1   //  Reboot the system
    #define PLAT_T_EVLOG_DUMP     2   //  Report events saved in event log
    #define PLAT_T_SOFT_REBOOT    3   //  Perform soft reboot, only reset states
    #define PLAT_T_TS_DUMP        4   //  Report task scheduler data

//  ID of this device when exchanging messages
const char DEVICE_ID[] = {"ROVER1"};

class Platform
{
    friend void _PLAT_KernelCallback(void);
    public:
        static Platform& GetI();
        static Platform* GetP();

        void InitHW();

        void Execute(const uint8_t* buf, const uint16_t len, int *err);

        //  Task scheduler is a requirement for platform
        volatile TaskScheduler *ts;

#ifdef __HAL_USE_ESP8266__
        ESP8266 *esp;
        DataStream telemetry;
        DataStream commands;
#endif
#ifdef __HAL_USE_ENGINES__
        EngineData *eng;
#endif
#ifdef __HAL_USE_MPU9250__
        MPU9250 *mpu;
#endif
#ifdef __HAL_USE_RADAR__
        RadarModule *rad;
#endif
    protected:
        Platform();
        ~Platform();

        void    _PostInit();

        //  Interface with task scheduler - provides memory space and function
        //  to call in order for task scheduler to request service from this module
        _kernelEntry _platKer;
};


#endif /* ROVERKERNEL_INIT_PLATFORM_H_ */
