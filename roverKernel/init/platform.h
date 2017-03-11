/**
 * platform.h
 *
 *  Created on: Mar 10, 2017
 *      Author: Vedran Mikov
 */

#ifndef ROVERKERNEL_INIT_PLATFORM_H_
#define ROVERKERNEL_INIT_PLATFORM_H_

#include "roverKernel/hwconfig.h"
#include "roverKernel/engines/engines.h"
#include "roverKernel/esp8266/esp8266.h"
#include "roverKernel/radar/radarGP2.h"
#include "roverKernel/mpu9250/mpu9250.h"
#include "roverKernel/taskScheduler/taskScheduler.h"
#include "roverKernel/serialPort/uartHW.h"

/**     Socket definitions for different data streams   */
/*
 * Telemetry data stream
 * Telemetry includes bidirectional stream starting from rover to server containing
 * sensor data, time reference, health report etc. On received frame from rover
 * server replies with "ACK\r\n"
 * Server expects telemetry stream on TCP port 2701
 */
#define P_TELEMETRY     0
/*
 * Commands data stream
 * This stream brings commands from server to rover. On received frame from
 * server rover replies "ACK\r\n"
 * Server expects commands stream on TCP port 2702
 */
#define P_COMMANDS      1


class Platform
{
    public:
        static Platform& GetI();
        static Platform* GetP();

        void InitHW();

#ifdef __HAL_USE_TASKSCH__
        volatile TaskScheduler* ts;
#endif
#ifdef __HAL_USE_ESP8266__
        ESP8266* esp;
#endif
#ifdef __HAL_USE_ENGINES__
        EngineData* eng;
#endif
#ifdef __HAL_USE_MPU9250__
        MPU9250* mpu;
#endif
#ifdef __HAL_USE_RADAR__
        RadarModule* rad;
#endif
    protected:
        Platform();
        ~Platform();
};


#endif /* ROVERKERNEL_INIT_PLATFORM_H_ */
