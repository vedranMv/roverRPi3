/**
 * hooks.h
 *
 *  Created on: Mar 10, 2017
 *      Author: Vedran Mikov
 *
 *  Definitions of all hook functions for various kernel modules used on the
 *  platform. These functions are usually hooked in on a certain events that
 *  occurred in kernel (such as network data was received). Usually they are
 *  never called from ISRs, but rather their execution is scheduled in ISR
 *  and their all called by the scheduler ASAP.
 *  @note Kernel allow for hot-swap of hooks, so it is possible to have multiple
 *  definitions here for a single hook (with different names). They can be
 *  passed to the kernel module before invoking a service in order to redirect
 *  service's output to a user-defined function.
 */

#ifndef ROVERKERNEL_INIT_HOOKS_H_
#define ROVERKERNEL_INIT_HOOKS_H_

#include "platform.h"
#include <string.h>
#include "libs/myLib.h"


/**
 * Function to be called when a new data is received from TCP clients on ALL
 * opened sockets at ESP. Function is called through data scheduler if enabled,
 * otherwise called directly from ISR
 * @param sockID socket ID at which the reply arrived
 * @param buf buffer containing incoming data
 * @param len size of incoming data in [buf] buffer
 */
static void ESPDataReceived(const uint8_t sockID, const uint8_t *buf, const uint16_t len)
{
    Platform &plat = Platform::GetI();
    //  Check which socket received data
    if (sockID == plat.telemetry.socketID)
    {
        //  Receiving data through this stream happens exclusively when there
        //  is a communication problem through 'commands' stream. Received
        //  data here triggers reboot of communications module
        plat.ts->SyncTask(ESP_UID, ESP_T_REBOOT, T_ASAP, false, 1);
        plat.ts->AddArg<uint8_t>(0x17);

    }
    else if (sockID == Platform::GetI().commands.socketID)
    {
        int err;
        uint8_t response[20] = {0};

        strcat((char*)response, DEVICE_ID);
        strcat((char*)response, ":");
        //  Parse incoming command and schedule its execution
        Platform::GetI().Execute(buf, len, &err);
        if (err == STATUS_OK)
            strcat((char*)response, "ACK\0");
        else
            strcat((char*)response, "NACK\0");

        Platform::GetI().commands.Send(response);
    }
    else
    {
        //  Nothing
    }
}

/**
 * Function called when a radar scan is completed, dumps data to a socket Id=0
 * if socket exists
 * @param scanData
 * @param scanLen
 */
static void RADScanComplete(uint8_t* scanData, uint16_t* scanLen)
{
    //  Once scan is completed schedule sending data through command socket
    TaskScheduler::GetP()->SyncTask(ESP_UID, ESP_T_SENDTCP, 0);
    TaskScheduler::GetP()->AddArg<uint8_t>(P_TO_SOCK(P_COMMANDS));  //Socket ID

    TaskScheduler::GetP()->AddArgs((void*)scanData, *scanLen);
}

#endif /* ROVERKERNEL_INIT_HOOKS_H_ */
