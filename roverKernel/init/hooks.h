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
    //  Print received data in hex or ASCII format
//    SerialPort::GetI().Send("%d says(%d):  %s", sockID, len, buf);
//    for (uint8_t i = 0; i < len; i++)
//        SerialPort::GetI().Send("0x%x ", buf[i]);
//    SerialPort::GetI().Send("\n");

    //  Check which socket received data
    if (sockID == Platform::GetI().telemetry.socketID)
    {
        //  Schedule new, fine radar scan at T+2s
        TaskScheduler::GetP()->SyncTask(RADAR_UID, RADAR_SCAN, -1000);
        TaskScheduler::GetP()->AddArg<uint8_t>(Platform::GetI().telemetry.socketID);
    }
    else if (sockID == Platform::GetI().commands.socketID)
    {
        //  Parse incoming command and schedule its execution
        Platform::GetI().DecodeIncoming(buf, len);
        uint32_t arg = (uint32_t)(&(Platform::GetI().commands));
        TaskScheduler::GetP()->RemoveTask(DATAS_UID, DATAS_T_KA, (void*)&arg, 4);
    }
    else
    {

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
    //  Once scan is completed schedule sending data to socket 0
    TaskScheduler::GetP()->SyncTask(ESP_UID, ESP_T_SENDTCP, 0);
    TaskScheduler::GetP()->AddArg<uint8_t>(1);  //Socket ID
    TaskScheduler::GetP()->AddArgs((void*)scanData, *scanLen);
}

#endif /* ROVERKERNEL_INIT_HOOKS_H_ */
