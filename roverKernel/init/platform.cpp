/**
 * platform.cpp
 *
 *  Created on: Mar 10, 2017
 *      Author: Vedran
 */
#include <roverKernel/init/hooks.h>
#include <roverKernel/init/platform.h>
#include "roverKernel/libs/myLib.h"

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
    //  If using task scheduler get handle and start systick every 100ms
#ifdef __HAL_USE_TASKSCH__
        ts = TaskScheduler::GetP();
        ts->InitHW(100);
#endif
    //  If using ESP chip, get handle and connect to access point
#ifdef __HAL_USE_ESP8266__
        esp = ESP8266::GetP();
        esp->InitHW();
        esp->AddHook(ESPDataReceived);
        esp->ConnectAP("sgvfyj7a", "7vxy3b5d");
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
}

///-----------------------------------------------------------------------------
///                      Class constructor & destructor              [PROTECTED]
///-----------------------------------------------------------------------------
Platform::Platform() {}
Platform::~Platform() {}
