/**
 * dataStream.cpp
 *
 *  Created on: Mar 16, 2017
 *      Author: Vedran
 */
#include "dataStream.h"

//  Makes sense to compile only if ESP module is being used
#if defined(__HAL_USE_ESP8266__)
#include "roverKernel/libs/myLib.h"
#include "roverKernel/esp8266/esp8266.h"

DataStream::DataStream(): _socketID(0), _port(0), _socket(0)
{
    memset((void*)_serverip, 0, sizeof(_serverip));
}

DataStream::DataStream(uint8_t *ip, uint16_t port)
    : _socketID(0), _port(port), _socket(0)
{
    uint8_t i;

    //  Find ip address length
    for (i = 0; ip[i] != 0; i++);
    memcpy((void*)_serverip, (void*)ip, i);
}


uint8_t DataStream::BindToScoketID(uint8_t sockID)
{
    //  Check if the socket is already opened
    _socket = ESP8266::GetI().GetClientBySockID(sockID);

    if (_socket!= 0)
        _socketID = sockID;
    else
    {   // Attempt to open a socket if IP address exists
        if (_serverip[0] == 0)
            return 222;
        _socketID = ESP8266::GetI().OpenTCPSock((char*)_serverip, _port, 1, sockID);
        //  Get reference to opened socket
        _socket = ESP8266::GetI().GetClientBySockID(sockID);
    }
    //  As a confirmation return socket id
    return _socketID;
}

/**
 * Send either a null terminated string with no buffer len, or any string
 * of a certain length
 * @note Wrapper for low-level espClient:: function
 * @param buffer
 * @param bufferLen
 */
void DataStream::Send(uint8_t *buffer, uint16_t bufferLen)
{
    //  Check if the socket is still opened
    _socket = ESP8266::GetI().GetClientBySockID(_socketID);
    if (_socket!= 0)
        return;

    _socket->SendTCP((char*)buffer, bufferLen);
}

/**
 * Receive data from the stream (if there's any)
 * @note Wrapper for low-level espClient:: function
 * @param buffer
 * @param bufferLen
 * @return
 */
bool DataStream::Receive(uint8_t *buffer, uint16_t *bufferLen)
{
    //  Check if the socket is still opened
    _socket = ESP8266::GetI().GetClientBySockID(_socketID);
    if (_socket!= 0)
        return;

    //  Check if there's response ready
    if (_socket->Ready())
    {
        //  Fetch response and save it into a buffer
        memcpy((void*)buffer, (void*)(_socket->RespBody), _socket->RespLen);
        (*bufferLen) = _socket->RespLen;
        _socket->Done();
        return true;
    }
    else
        return false;
}


#endif /* __HAL_USE_ESP8266__ */

