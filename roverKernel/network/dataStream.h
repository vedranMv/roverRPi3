/**
 * dataStream.h
 *
 *  Created on: Mar 16, 2017
 *      Author: Vedran Mikov
 *
 *  Data stream object that utilizes sockets handled by the ESP module. Maintains
 *  a data stream with a server that allows exchanging data. Data can be any
 *  uint8_t character and is not limited to readable range (0x00 - 0x7F). Library
 *  can be integrated with task scheduler to periodically check if the stream is
 *  opened and try to reconnect in case of a failure.
 *
 *  @version 1.2
 *  V1.0 - 17.3.2017
 *  +Created document
 *  +Functionality: Initialize data stream with server IP & port, bind to opened
 *  socket or attempt to open new socket
 *  V1.1 - 23.3.2017
 *  +When sending, if socket is closed, attempt to reopen and rebind it to stream
 *  V1.2 - 24.3.2017
 *  +On first call to BindToSocketID() object registers periodic health-check of
 *  connection to server. Attempts to reestablish lost connection.
 */
#include "roverKernel/hwconfig.h"

//  Makes sense to compile only if ESP module is being used
#if !defined(ROVERKERNEL_NETWORK_DATASTREAM_H_) && defined(__HAL_USE_ESP8266__)
#define ROVERKERNEL_NETWORK_DATASTREAM_H_

#include "roverKernel/esp8266/espClient.h"

//  Enable integration of this library with task scheduler but only if task
//  scheduler is being compiled into this project
#if defined(__HAL_USE_TASKSCH__)
#define __USE_TASK_SCHEDULER__
#endif  /* __HAL_USE_TASKSCH__ */

//  Check if this library is set to use task scheduler
#if defined(__USE_TASK_SCHEDULER__)
    #include "roverKernel/taskScheduler/taskScheduler.h"
    //  Unique identifier of this module as registered in task scheduler
    #define DATAS_UID       4
    //  Definitions of ServiceID for service offered by this module
    #define DATAS_T_KA      0   //  Keep alive socket

//  Function to register data stream as a kernel module into the task scheduler
void DataStream_InitHW();

#endif

/**
 * Definition of DataStream class. High level network communication object that
 * utilizes network sockets handled by ESP8266 library to establish a two-way
 * data stream with TCP server. Once the data stream has been bounded to a
 * socket it maintains the connectivity through periodic health-checks (using
 * task scheduler) and attempts reconnect in case of lost connection.
 */
class DataStream
{
    friend void _DATAS_KernelCallback(void);
    public:
        DataStream();
        DataStream(uint8_t *ip, uint16_t port);
        ~DataStream();

        uint8_t BindToSocketID(uint8_t sockID);

        void    Send(uint8_t *buffer, uint16_t bufferLen = 0);
        bool    Receive(uint8_t *buffer, uint16_t *bufferLen);

        //  Socket ID as returned from ESP8266
        uint8_t     socketID;

    private:
        //  String containing server IP address of underlying socket
        uint8_t     _serverip[20];
        //  Port number of server to which this stream is opened
        uint16_t    _port;
        //  Socket handle
        _espClient* _socket;
        //  Turns true once this data stream has scheduled periodic checking
        //  of socket's health (whether we're still connected to the server)
        bool        _keepAlive;
};



#endif /* ROVERKERNEL_NETWORK_DATASTREAM_H_ */
