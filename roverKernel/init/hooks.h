/**
 * hooks.h
 *
 *  Created on: Mar 10, 2017
 *      Author: Vedran Mikov
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
void ESPDataReceived(uint8_t sockID, uint8_t *buf, uint16_t *len)
{
    switch(sockID)
    {
    case P_TELEMETRY:
        {
            //  Assemble telemetry frame
            //  [internalTime]:[]

        }
        break;
    case P_COMMANDS:
        {
            //  Schedule new, fine radar scan at T+2s
            TaskScheduler::GetP()->SyncTask(RADAR_UID, RADAR_SCAN, -1000);
            TaskScheduler::GetP()->AddArg<uint8_t>(1);
        }
        break;
    default:
        break;
    }
    //  Print received data
    SerialPort::GetI().Send("%d says:  %s", sockID, buf);


}

/**
 * Function called when a radar scan is completed, dumps data to a socket Id=0
 * if socket exists
 * @param scanData
 * @param scanLen
 */
void RADScanComplete(uint8_t* scanData, uint16_t* scanLen)
{
    //  Once scan is completed schedule sending data to socket 0
    TaskScheduler::GetP()->SyncTask(ESP_UID, ESP_T_SENDTCP, 0);
    TaskScheduler::GetP()->AddArg<uint8_t>(1);  //Socket ID
    TaskScheduler::GetP()->AddArgs((void*)scanData, *scanLen);
}

#endif /* ROVERKERNEL_INIT_HOOKS_H_ */
