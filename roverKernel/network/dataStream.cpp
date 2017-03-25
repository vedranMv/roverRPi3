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
#include "roverKernel/serialPort/uartHW.h"


#if defined(__USE_TASK_SCHEDULER__)
/*
 * Interface with task scheduler - provides memory space and function to call in
 * order for task scheduler to request service from this module. Not defined
 * within the class as it's enough to have only one of these to check all opened
 * streams.
 */
static _kernelEntry _dsKer;

/**
 * Callback routine to invoke service offered by this module from task scheduler
 * @note It is assumed that once this function is called task scheduler has
 * already copied required variables into the memory space provided for it.
 */
void _DATAS_KernelCallback(void)
{
    //  Check for null-pointer
    if (_dsKer.argN == 0)
        return;
    /*
     *  Data in args[] contains bytes that constitute arguments for function
     *  calls. The exact representation(i.e. whether bytes represent ints, floats)
     *  of data is known only to individual blocks of switch() function. There
     *  is no predefined data separator between arguments inside args[].
     */
    switch (_dsKer.serviceID)
    {
    /*
     * Keep-alive event, check if the socket is still alive, if not try to reconnect
     * args[] = pointerToDatastreamObject(DataStream*)
     * retVal ESP library error code (if > 5); or socket ID (if <=5)
     */
    case DATAS_T_KA:
        {
            //  Pointer is encoded into integer number
            uint32_t ptr = 0;
            memcpy(&ptr, (void*)_dsKer.args, 4);
            DataStream *ds = (DataStream*)ptr;

            _espClient *socket = ESP8266::GetI().GetClientBySockID(ds->socketID);

            /*
             * If socket has been closed GetClientBySockID returns 0. To reopen
             * it we just call BindToScoketID as it already handles that
             */
            if (socket == 0)
                ds->BindToSocketID(ds->socketID);
        }
        break;
    default:
        break;
    }

}

/**
 * Registers data stream as a kernel module if compiled with task scheduler
 */
void DataStream_InitHW()
{
    //  Register module services with task scheduler
    _dsKer.callBackFunc = _DATAS_KernelCallback;
    TS_RegCallback(&_dsKer, DATAS_UID);
}

#endif



///-----------------------------------------------------------------------------
///                      Class constructor & destructor                 [PUBLIC]
///-----------------------------------------------------------------------------
DataStream::DataStream(): socketID(0), _port(0), _socket(0), _keepAlive(false)
{
    memset((void*)_serverip, 0, sizeof(_serverip));
}

DataStream::DataStream(uint8_t *ip, uint16_t port)
    : socketID(0), _port(port), _socket(0), _keepAlive(false)
{
    uint8_t i;

    //  Find ip address length
    for (i = 0; ip[i] != 0; i++);
    memcpy((void*)_serverip, (void*)ip, i);
}

DataStream::~DataStream()
{
    if (_keepAlive)
    {
        //  Delete periodic task attempting to reconnect to server
        uint32_t arg = (uint32_t)this;
        TaskScheduler::GetP()->RemoveTask(DATAS_UID, DATAS_T_KA, (void*)&arg, sizeof(uint32_t));
    }
    //  Close the socket before deleting data stream
    _socket->Close();
}

///-----------------------------------------------------------------------------
///                      Public member functions                        [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Bind data stream to a specific socket id. If socket already exists it just
 * binds it, if it doesn't it attempts to open it with first free ID and bind
 * data stream to that ID. Socket id bound to this stream is saved internally
 * and returned from this function.
 * @param sockID socket ID as returned from ESP chip (0 - 4) of a socket to bind to
 * @return socket ID to which data stream was eventually bound; 127 on error
 */
uint8_t DataStream::BindToSocketID(uint8_t sockID)
{

#if defined(__USE_TASK_SCHEDULER__)
    if (!_keepAlive)
    {
    //  Schedule periodic check for health of the underlying socket, period 4s
    TaskScheduler::GetI().SyncTask(DATAS_UID, DATAS_T_KA, -4000, true, T_PERIODIC);
    TaskScheduler::GetI().AddArg<uint32_t>((uint32_t)this);
    _keepAlive = true;
    }
#endif

    //  Check if the socket is already opened
    _socket = ESP8266::GetI().GetClientBySockID(sockID);

    if (_socket!= 0)
        socketID = sockID;
    else
    {   // Attempt to open a socket if IP address exists
        if (_serverip[0] == 0)
            return 222;
        //  Attempt to open the socket and check for error codes (> max clients)
        uint32_t status;
        status = ESP8266::GetI().OpenTCPSock((char*)_serverip, _port, 1, sockID);
        if (status < ESP_MAX_CLI)
            socketID = status;
        else
            return 127;
        //  Get reference to opened socket
        _socket = ESP8266::GetI().GetClientBySockID(sockID);
    }
    //  As a confirmation return socket id
    return socketID;
}

/**
 * Send either a null terminated string with no buffer len, or any string of a
 * certain length through the stream. Function checks whether the bounded socket
 * is still alive, if not tries to reopen it.
 * @note Wrapper for low-level espClient:: function
 * @param buffer
 * @param bufferLen
 */
void DataStream::Send(uint8_t *buffer, uint16_t bufferLen)
{
    //  Check if the socket is still opened
    _socket = ESP8266::GetI().GetClientBySockID(socketID);
    if (_socket!= 0)
    {
        //  If it isn't attempt to reopen it
        BindToSocketID(socketID);
    }

    _socket->SendTCP((char*)buffer, bufferLen);
}

/**
 * Receive data from the stream (if there's any)
 * @note Wrapper for low-level espClient:: function
 * @param buffer
 * @param bufferLen
 * @return true: if new data was put into buffer, false otherwise
 */
bool DataStream::Receive(uint8_t *buffer, uint16_t *bufferLen)
{
    //  Check if the socket is still opened, if it isn't there's no use in
    //  reopening it as there will be no new data to read; so just return
    _socket = ESP8266::GetI().GetClientBySockID(socketID);
    if (_socket!= 0)
        return false;

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
