/**
 * dataStream.h
 *
 *  Created on: Mar 16, 2017
 *      Author: Vedran Mikov
 *
 *  Data stream object that utilizes sockets handled by the ESP module
 *
 *  @version 1.0
 *  V1.0
 *  +Created document
 *
 */
#include "roverKernel/hwconfig.h"

//  Makes sense to compile only if ESP module is being used
#if !defined(ROVERKERNEL_NETWORK_DATASTREAM_H_) && defined(__HAL_USE_ESP8266__)
#define ROVERKERNEL_NETWORK_DATASTREAM_H_
#include "roverKernel/esp8266/espClient.h"


class DataStream
{
    public:
        DataStream();
        DataStream(uint8_t *ip, uint16_t port);

        uint8_t BindToScoketID(uint8_t sockID);

        void    Send(uint8_t *buffer, uint16_t bufferLen = 0);
        bool    Receive(uint8_t *buffer, uint16_t *bufferLen);

    private:

        uint8_t     _socketID;
        uint8_t     _serverip[20];
        uint16_t    _port;
        _espClient* _socket;
};



#endif /* ROVERKERNEL_NETWORK_DATASTREAM_H_ */
