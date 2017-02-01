/**
 * esp8266.cpp
 *
 *  Created on: 1. 8. 2016.
 *      Author: Vedran Mikov
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include "driverlib/uart.h"

#include "myLib.h"
#include "esp8266.h"
#include "utils/uartstdio.h"
#include "tm4c1294_hal.h"

#if defined(__USE_TASK_SCHEDULER__)

/**
 * Callback routine to invoke service offered by this module from task scheduler
 * @note It is assumed that once this function called task scheduler has already
 * copied required variables into the memory space provided for it.
 */
void _ESP_KernelCallback(void)
{
    /*
     *  Data in args[] array always has first byte(args[0]) containing the size
     *  of the args[] array (without first byte). So total size is args[0]+1.
     *  First data byte is accessed at args[0]
     */
    switch (__esp->_espSer.serviceID)
    {
    /*
     * Start/Stop control for TCP server
     * 1st data byte of args[] is either 0(stop) or 1(start). Following bytes
     * 2 & 3 contain uint16_t value of port at which to start server
     * args[] = size(1B)|enable(1B)|port(2B)\0
     * retVal ESP library status code
     */
    case ESP_T_TCPSERV:
        {
            if (__esp->_espSer.args[1] == 1)
            {
                uint16_t port;
                memcpy((void*)&port, (void*)(__esp->_espSer.args + 2), 2);
                __esp->_espSer.retVal = __esp->StartTCPServer(port);
                __esp->TCPListen(true);
            }
            else
            {
                __esp->_espSer.retVal = __esp->StopTCPServer();
                __esp->TCPListen(false);
            }
        }
        break;
    /*
     * Connect to TCP client on given IP address and port
     * args[] = size(1B)|KeepAlive(1B)|IPaddress(7B-15B)|port(2B)|\0
     * retVal ESP library error code (if > 5); or socket ID (if <=5)
     */
    case ESP_T_CONNTCP:
        {
            char ipAddr[15] = {0};
            uint16_t port;
            //  IP address starts on 2nd data byte and its a string of length
            //  equal to total length of data - 3bytes(used for port & KA)
            memcpy( (void*)ipAddr,
                    (void*)(__esp->_espSer.args + 2),
                    __esp->_espSer.args[0] - 3);
            //  Port is last two bytes
            memcpy( (void*)&port,
                    (void*)(__esp->_espSer.args + (__esp->_espSer.args[0]-1)),
                    2);
            //  If IP address is valid process request
            if (__esp->_IPtoInt(ipAddr) == 0)
                return;
            //  1st data byte is keep alive flag
            //  Double negation to convert any integer into boolean
            bool KA = !(!__esp->_espSer.args[1]);
            __esp->_espSer.retVal = __esp->OpenTCPSock(ipAddr, port, KA);
        }
        break;
    /*
     * Send message to specific TCP client
     * args[] = size(1B)|socketID(1B)|message|\0
     */
    case ESP_T_SENDTCP:
        {
            //  Check if socket ID is valid
            if (!__esp->ValidSocket(__esp->_espSer.args[1]))
               return;
            //  Ensure that message is null-terminated
            if ((__esp->_espSer.args[0] + 1) < TS_TASK_MEMORY)
                __esp->_espSer.args[__esp->_espSer.args[0]+1] = '\0';
            else
                __esp->_espSer.args[49] = '\0';
            //  Initiate TCP send to required client
            __esp->_espSer.retVal = __esp->GetClientBySockID(__esp->_espSer.args[1])
                                      ->SendTCP((char*)(__esp->_espSer.args+2));
        }
        break;
    /*
     * Receive data from an opened socket and send it to user-defined routine
     * args[] = size(1B)|socketID(1B)\0
     */
    case ESP_T_RECVSOCK:
        {
            _espClient  *cli;
            //  Check if socket ID is valid
            if (!__esp->ValidSocket(__esp->_espSer.args[1]))
                return;
            cli = __esp->GetClientBySockID(__esp->_espSer.args[1]);
            __esp->custHook(__esp->_espSer.args[1],
                            (uint8_t*)(cli->RespBody),
                            (uint16_t*)(&(cli->RespLen)));
        }
        break;
    /*
     * Close socket with specified ID
     * args[] = size(1B)|socketID(1B)\0
     */
    case ESP_T_CLOSETCP:
        {
            //  Check if socket ID is valid
            if (!__esp->ValidSocket(__esp->_espSer.args[1]))
                return;
            //  Initiate closing from client object
            __esp->_espSer.retVal = __esp->GetClientBySockID(__esp->_espSer.args[1])
                                              ->Close();
        }
        break;
    default:
        break;
    }

}
#endif

/**
 * Routine invoked by watchdog timer on timeout
 * Sets global status for current communication to "error", clears WD interrupt
 * flag and CALLS the ESP's interrupt handler to process any remaining data in there
 */
void ESPWDISR()
{
    __esp->flowControl = ESP_STATUS_ERROR;
    HAL_ESP_WDClearInt();
}

/*      Lookup table for statuses returned by ESP8266       */
/*const char status_table[][20]={ {"OK"}, {"BUSY"}, {"ERROR"}, {"NONBLOCKING"},
                                {"CONNECTED"}, {"DISCONNECTED"}, {"READY"},
                                {"SOCK_OPEN"}, {"SOCK_CLOSED"}, {"RECEIVE"},
                                {"FAILED"}, {"SEND OK"}, {"SUCCESS"}};*/

//  Dummy instance used for return value when no available client
_espClient dummy;

//  Pointer to first created instance of ESP8266
ESP8266* __esp;

//  Buffer used to assemble commands
char _commBuf[512];

//  Dummy function to be called to suppress "Unused variable" warnings
void UNUSED(int32_t arg) { }

/*******************************************************************************
 *******************************************************************************
 *********            _espClient class member functions                *********
 *******************************************************************************
 ******************************************************************************/

///-----------------------------------------------------------------------------
///                      Class constructor & destructor                [PUBLIC]
///-----------------------------------------------------------------------------
_espClient::_espClient() : KeepAlive(true), _parent(0), _id(0) ,_alive(false)
{
    _Clear();
}

_espClient::_espClient(uint8_t id, ESP8266 *par)
    : KeepAlive(true), _parent(par), _id(id), _alive(true)
{
    _Clear();
}
_espClient::_espClient(const _espClient &arg)
    : KeepAlive(arg.KeepAlive), _parent(arg._parent), _id(arg._id), _alive(arg._alive)
{
    _Clear();
}

void _espClient::operator= (const _espClient &arg)
{
    _parent = arg._parent;
    _id = arg._id;
    _alive = arg._alive;
    _respRdy = arg._respRdy;
    KeepAlive = arg.KeepAlive;
    memcpy((void*)RespBody, (void*)(arg.RespBody), sizeof(RespBody));
}

///-----------------------------------------------------------------------------
///                      Client socket-manipulation                     [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Send data to a client over open TCP socket
 * @param buffer NULL-TERMINATED(!) data to send
 * @return status of send process (binary or of ESP_* flags received while sending)
 */
uint32_t _espClient::SendTCP(char *buffer)
{
    uint16_t bufLen = 0;

    while (buffer[bufLen++] != '\0');   //Find length of string
    bufLen--;
    //  Initiate transmission from
    snprintf(_commBuf, sizeof(_commBuf), "AT+CIPSEND=%d,%d\0",_id, bufLen);
    if (_parent->_SendRAW(_commBuf, ESP_STATUS_RECV))
    {
        _parent->flowControl = ESP_NO_STATUS;
        if (!_parent->_servOpen) HAL_ESP_IntEnable(true);
        _parent->_RAWPortWrite(buffer, bufLen);
        //  Listen for potential response

        while (_parent->flowControl == ESP_NO_STATUS);
    }

    return _parent->flowControl;
}
/**
 * Read response from client saved in internal buffer
 * Internal buffer with response is filled as soon as response is received in
 * an interrupt. This function only copies response from internal buffer to a
 * user provided one and then clears internal (and data-ready flag).
 * @param buffer pointer to user-provided buffer for incoming data
 * @param bufferLen used to return the buffer size to user
 * @return ESP_NO_STATUS: on success,
 *         ESP_NORESPONSE: if no response is available
 */
uint32_t _espClient::Receive(char *buffer, uint16_t *bufferLen)
{
    (*bufferLen) = 0;
    //  Check if there's new data received
    if (_respRdy)
    {
        //  Fill argument buffer
        while(RespBody[(*bufferLen)] != 0)
        {
            buffer[(*bufferLen)] = RespBody[(*bufferLen)];
            (*bufferLen)++;
        }

        //  Clear response body & flag
        _Clear();

        //  Check if it's supposed to stay open, if not force closing or schedule
        //  closing(preferred) of socket
        if (!KeepAlive)
        {
#if defined(__USE_TASK_SCHEDULER__)
            __taskSch->SyncTask(ESP_UID, ESP_T_CLOSETCP, 0);
            __taskSch->AddStringArg(&_id, 1);
#else
            Close();
#endif
        }

        return ESP_NO_STATUS;
    }
    else return ESP_NORESPONSE;
}

/**
 * Check is socket has any new data ready for user
 * @note used when manually reading response body from member variable to check
 * whether new data is available. If used, Done() MUST be called when done
 * processing data in response body. Alternative: use Receive() function instead
 * @return true: if there's new data from that socket
 *         false: otherwise
 */
bool _espClient::Ready()
{
    return _respRdy;
}

/**
 * Clears response body, flags and maintains socket alive if specified
 * @note has to be called if user manually reads response body by reading member
 * variable directly, and not through Receive() function call
 */
void _espClient::Done()
{
    //  Clear response body & flag
    _Clear();
    //  Check if it's supposed to stay open, if not force closing or schedule
    //  closing(preferred) of socket
    if (!KeepAlive)
    {
#if defined(__USE_TASK_SCHEDULER__)
        __taskSch->SyncTask(ESP_UID, ESP_T_CLOSETCP, 0);
        __taskSch->AddStringArg(&_id, 1);
#else
        Close();
#endif
    }
}

/**
 * Force closing TCP socket with the client
 * @note Object is deleted in ParseResponse function, once ESP confirm closing
 * @return status of close process (binary or of ESP_* flags received while closing)
 */
uint32_t _espClient::Close()
{
    snprintf(_commBuf, sizeof(_commBuf), "AT+CIPCLOSE=%d\0", _id);
    _alive = false;
    return _parent->_SendRAW(_commBuf);
}

/**
 * Clear response body and flag for response ready
 */
void _espClient::_Clear()
{
    memset((void*)RespBody, 0, sizeof(RespBody));
    RespLen = 0;
    _respRdy = false;
}
/*******************************************************************************
 *******************************************************************************
 *********              ESP8266 class member functions                 *********
 *******************************************************************************
 ******************************************************************************/

///-----------------------------------------------------------------------------
///                      Class constructor & destructor                [PUBLIC]
///-----------------------------------------------------------------------------

ESP8266::ESP8266() : custHook(0), flowControl(ESP_NO_STATUS), _tcpServPort(0),
                     _ipAddress(0), _servOpen(false)
{
    if (__esp == 0) __esp = this;
}

ESP8266::~ESP8266()
{
    Enable(false);
    __esp = 0;
}

///-----------------------------------------------------------------------------
///         Public functions used for configuring ESP8266               [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Initialize UART port used for ESP module. Also enable watchdog timer used to
 * reset the port in case of any errors or hangs
 * @param baud baud-rate used in serial communication between ESP and hardware
 * @return error code, depending on the outcome
 */
uint32_t ESP8266::InitHW(int32_t baud)
{
    HAL_ESP_InitPort(baud);
    HAL_ESP_RegisterIntHandler(UART7RxIntHandler);
    HAL_ESP_InitWD(ESPWDISR);

    //    Turn ESP8266 chip ON
    Enable(true);

    //  Send test command(AT) then turn off echoing of commands(ATE0)
    _SendRAW("AT\0");
    _SendRAW("ATE0\0");

#if defined(__USE_TASK_SCHEDULER__)
    //  Register module services with task scheduler
    _espSer.callBackFunc = _ESP_KernelCallback;
    TS_RegCallback(&_espSer, ESP_UID);
#endif

    return ESP_STATUS_OK;
}

/**
 * Enable/disable ESP8266 chip (by controlling CH_PD pin)
 * @param enable new state of chip to set
 */
void ESP8266::Enable(bool enable)
{
    HAL_ESP_HWEnable(enable);
    while (!HAL_ESP_IsHWEnabled());
}

/**
 * Check if ESP chip is enabled (CH_PD pin pulled high)
 * @return true if chip is enabled, false if not
 */
bool ESP8266::IsEnabled()
{
    return HAL_ESP_IsHWEnabled();
}

/**
 * Register hook to user function
 * Register hook to user-function called every time new data from TCP/UDP client
 * is received. Received data is passed as an argument to hook function.
 * @param funPoint pointer to void function with 3 arguments
 */
void ESP8266::AddHook(void((*funPoint)(uint8_t, uint8_t*, uint16_t*)))
{
    custHook = funPoint;
}

///-----------------------------------------------------------------------------
///                  Functions used with access points                  [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Connected to AP using provided credentials
 * @param APname name of AP to connect to
 * @param APpass password of AP connecting to
 * @return
 */
uint32_t ESP8266::ConnectAP(char* APname, char* APpass)
{
    int8_t retVal = ESP_NO_STATUS;
    //char command[100];

    //  Set ESP in client mode
    retVal = _SendRAW("AT+CWMODE_CUR=1\0");
    if (!_InStatus(retVal, ESP_STATUS_OK)) return retVal;

    //  Assemble command & send it
    snprintf(_commBuf, sizeof(_commBuf), "AT+CWJAP_CUR=\"%s\",\"%s\"\0", APname, APpass);
    //  Use standard send function but increase timeout to 4s as acquiring IP
    //  address might take time
    retVal = _SendRAW(_commBuf, 0, 6000);
    if (!_InStatus(retVal, ESP_STATUS_OK)) return retVal;

    //  Acquire IP address and save it locally
    MyIP();

    //  Allow for multiple connections, in case of error return
    retVal = _SendRAW("AT+CIPMUX=1\0");
    if (!_InStatus(retVal, ESP_STATUS_OK)) return retVal;

    return retVal;
}

/**
 * Check if ESP is connected to AP (if it acquired IP address)
 * @return true if it's connected, false otherwise
 */
bool ESP8266::IsConnected()
{
    return (_ipAddress > 0);
}

/**
 * Get IP address assigned to device when connected to AP
 * @return
 */
uint32_t ESP8266::MyIP()
{
    if (_ipAddress == 0) _SendRAW("AT+CIPSTA?\0");

    return _ipAddress;
}

/**
 * Disconnect from WiFi Access point
 * @return
 */
uint32_t ESP8266::DisconnectAP()
{
    //  Release internal IP address
    memset(_ipStr, 0, sizeof(_ipStr));
    _ipAddress = 0;

    return _SendRAW("AT+CWQAP\0");
}

///-----------------------------------------------------------------------------
///                      Functions related to TCP server                [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Initiate TCP server on given port number
 * @param port at which to start listening for incoming connections
 * @return error code, depending on the outcome
 */
uint32_t ESP8266::StartTCPServer(uint16_t port)
{
    int8_t retVal = ESP_STATUS_OK;

    //  Start TCP server, in case of error return
    snprintf(_commBuf, sizeof(_commBuf), "AT+CIPSERVER=1,%d\0", port);
    retVal = _SendRAW(_commBuf);
    if (!_InStatus(retVal, ESP_STATUS_OK)) return retVal;

    _tcpServPort = port;
    _servOpen = true;

    //  Set TCP connection timeout to 0, in case of error return
    retVal = _SendRAW("AT+CIPSTO=0\0");
    if (!_InStatus(retVal, ESP_STATUS_OK)) return retVal;

    return retVal;
}

/**
 * Configure listening for incoming data when running server mode OR waiting for
 * response on opened TCP socket
 * @param enable set true for initiating listening, false otherwise
 */
void ESP8266::TCPListen(bool enable)
{
    HAL_ESP_IntEnable(enable);
    _servOpen = enable;
}

/**
 * Check if TCP server is running
 * @return true if server is running, false if not
 */
bool ESP8266::ServerOpened()
{
    return _servOpen;
}

/**
 * Stop TCP server running on ESP
 * @return error code depending on the outcome
 */
uint32_t ESP8266::StopTCPServer()
{
    int8_t retVal = ESP_STATUS_OK;

    //  Stop TCP server, in case of error return
    retVal = _SendRAW("AT+CIPSERVER=0");
    if (!_InStatus(retVal, ESP_STATUS_OK)) return retVal;

    _servOpen = false;
    _tcpServPort = 0;
    return retVal;
}

///-----------------------------------------------------------------------------
///                      Functions related to TCP clients               [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Open TCP socket to a client at specific IP and port, keep alive interval 7.2s
 * @param ipAddr string containing IP address of server
 * @param port TCP socket port of server
 * @return On success socket ID of TCP client in _client vector, on failure
 *         ESP_STATUS_ERROR error code
 */
uint32_t ESP8266::OpenTCPSock(char *ipAddr, uint16_t port, bool keepAlive)
{
    uint32_t retVal;
    uint8_t sockID;
    //  Find free socket number (0-4 supported)
    for (sockID = 0; sockID < 4; sockID++)
    {
        bool used = false;
        for (uint8_t j = 0; j < _clients.size(); j++)
            if (_clients[j]._id == sockID) used = true;
        if (!used) break;
    }

    //  Assemble command: Open TCP socket to specified IP and port, set
    //  keep alive interval to 7200ms
    snprintf(_commBuf, sizeof(_commBuf), "AT+CIPSTART=%d,\"TCP\",\"%s\",%d,7200",
             sockID, ipAddr, port);

    //  Execute command and check outcome
    retVal = _SendRAW(_commBuf);
    if (_InStatus(retVal, ESP_STATUS_OK))
    {
        //  If success, start listening for potential incoming data from server
        TCPListen(true);
        GetClientBySockID(sockID)->KeepAlive = keepAlive;
        retVal = sockID;
    }

    return retVal;
}

/**
 * Check if socket with the following ID is open (alive)
 * @param id ID of the socket to check
 * @return alive status of particular socket
 */
bool ESP8266::ValidSocket(uint16_t id)
{
    return GetClientBySockID(id)->_alive;
}

/**
 * Get pointer to client object on specific index in _client vector
 * @param index desired index of client to get
 * @return pointer to ESP client object under given index (if it exists, if not
 *          dummy returned)
 */
_espClient* ESP8266::GetClientByIndex(uint16_t index)
{
    if (_clients.size() > index)
    {
        if (_clients[index]._alive)
            return &_clients[index];
        else
        {
            _clients.erase(_clients.begin() + index);
            return &dummy;
        }
    }
    else
        return &dummy;
}

/**
 * Get pointer to client object on specific socket id
 * @param id socket id as returned by ESP on opened socket
 * @return pointer to ESP client object under given id (if it exists, if not
 *          dummy returned)
 */
_espClient* ESP8266::GetClientBySockID(uint16_t id)
{
    uint16_t index = _IDtoIndex(id);
    return GetClientByIndex(index);
}


/**
 * Send a message to TCP client on socket ID 0 (if exists)
 * @param arg string message to send
 * @return error code, depending on the outcome
 */
uint32_t ESP8266::Send2(char* arg)
{
    //  Check if client exists then try and send request from it
    if (!_clients[_IDtoIndex(0)]._alive) return ESP_STATUS_FAIL;

    return _clients[_IDtoIndex(0)].SendTCP(arg);
}

/**
 * @brief ESP reply message parser
 * Checks ESP reply stream for commands, actions and events and updates global
 * variables accordingly.
 * @param rxBuffer string containing reply message from ESP
 * @param rxLen length of rxBuffer string
 * @return bitwise OR of all statuses(ESP_STATUS_*) found in the message string
 */
uint32_t ESP8266::ParseResponse(char* rxBuffer, uint16_t rxLen)
{
    //  Return value
    uint32_t retVal = ESP_NO_STATUS;
    //  Flags to be set when the parameter has been found
    int16_t ipFlag= -1, respFlag = -1, sockOflag = -1, sockCflag = -1;
    //  Pointer to char, used to temporary store return value of strstr function
    char *retTemp;

    if (rxLen < 1)    return retVal;

    //  IP address embedded, save pointer to its first digit(as string)
    if ((retTemp = strstr(rxBuffer,"ip:\"")) != NULL)
        ipFlag = retTemp - rxBuffer + 4;

    //  Message from one of the sockets, save pointer to first character
    //  (which is always message length!)
    if ((retTemp = strstr(rxBuffer,"+IPD,")) != NULL)
    {
        respFlag = retTemp - rxBuffer + 5;
        retVal |= ESP_STATUS_IPD;
    }

    //  Look for general status messages returned by ESP
    if (strstr(rxBuffer,"OK") != NULL)
        retVal |= ESP_STATUS_OK;
    if (strstr(rxBuffer,"busy...") != NULL)
        retVal |= ESP_STATUS_BUSY;
    if (strstr(rxBuffer,"FAIL") != NULL)
        retVal |= ESP_STATUS_FAIL;
    if (strstr(rxBuffer,"ERROR") != NULL)
        retVal |= ESP_STATUS_ERROR;
    if (strstr(rxBuffer,"READY") != NULL)
        retVal |= ESP_STATUS_READY;

    if (strstr(rxBuffer,"WIFI CONN") != NULL)
        retVal |= ESP_STATUS_CONNECTED;
    if (strstr(rxBuffer,"WIFI DISCONN") != NULL)
        retVal |= ESP_STATUS_CONNECTED;

    if (strstr(rxBuffer,"SEND OK") != NULL)
        retVal |= ESP_STATUS_SENDOK;
    if (strstr(rxBuffer,"SUCCESS") != NULL)
        retVal |= ESP_RESPOND_SUCC;
    if (strstr(rxBuffer,">") != NULL)
        retVal |= ESP_STATUS_RECV;

    //  If new socket is opened save socket id
    if ((retTemp =strstr(rxBuffer,",CONNECT")) != NULL)
    {
        sockOflag = retTemp - rxBuffer - 1; //Sock ID position
        retVal |= ESP_STATUS_SOCKOPEN;
    }
    //  If socket is closed save socket id
    if ((retTemp =strstr(rxBuffer,",CLOSED")) != NULL)
    {
        sockCflag = retTemp - rxBuffer - 1; //Sock ID position
        retVal |= ESP_STATUS_SOCKCLOSE;
    }

    //  IP address embedded, extract it
    if (ipFlag >= 0)
    {
        int i = ipFlag;
        memset(_ipStr, 0, 16);
        while(((rxBuffer[i] == '.') || isdigit(rxBuffer[i])) && (i < rxLen))
            { _ipStr[i-ipFlag] = rxBuffer[i]; i++; }

        _ipAddress = _IPtoInt(_ipStr);
        retVal |= ESP_GOT_IP;
    }
    //  TCP incoming data embedded, extract it
    //  Data format:> +IPD,socketID,length:message
    if (respFlag >= 0)
    {
        int i;
        //  Get client who sent the incoming data (based on socket ID)
        _espClient *cli = &_clients[_IDtoIndex(rxBuffer[respFlag] - 48)];

        //  i points to socket ID
        i = respFlag+2; //Skip colon and go to first digit of length
        //  Double dot marks beginning of the message; also extract length of it
        uint8_t cmsgLen[3] = {0};
        while(rxBuffer[i++] != ':') //  ++ here so double dot is skipped when done
            cmsgLen[i-1-respFlag-2] = rxBuffer[i-1];
        cli->RespLen = (uint16_t)lroundf(stof(cmsgLen, i-1-respFlag));

        //  Use raspFlag to mark starting point
        respFlag = i;
        //  Loop until the end of received message
        while((i-respFlag) < cli->RespLen)
        {
            cli->RespBody[i - respFlag] = rxBuffer[i];
            i++;
        }
        cli->_respRdy = true;
    }
    //  Socket got opened, create new client for it
    if (sockOflag >= 0)
        _clients.push_back(_espClient(rxBuffer[sockOflag] - 48, this));
    //  Socket got closed, find client with this ID and delete it
    if (sockCflag >= 0)
        _clients.erase(_clients.begin()+_IDtoIndex(rxBuffer[sockCflag] - 48));

    return retVal;
}

/**
 * @brief Check whether the flags are set in the status message
 * @param status to check for flags
 * @param flag bitwise OR of ESP_STATUS_* values to look for in status
 * @return true: if ALL flags are set in status
 *         false: if at least on flag is not set in status
 */
bool ESP8266::_InStatus(const uint32_t status, const uint32_t flag)
{
    return ((status & flag) > 0);
}

/**
 * @brief Send command to ESP8266 module
 * Sends command passed in the null-terminated txBuffer. This is a blocking
 * function, awaiting reply from ESP. Function returns when status OK or ERROR
 * or any other status passed in flags have been received from ESP.
 * @param txBuffer null-terminated string with command to execute
 * @param flags bitwise OR of ESP_STATUS_* values
 * @return bitwise OR of ESP_STATUS_* returned by the ESP module
 */
uint32_t ESP8266::_SendRAW(const char* txBuffer, uint32_t flags, uint32_t timeout)
{
    uint16_t txLen = 0;

    //  Reset global status
    flowControl = ESP_NO_STATUS;
    //  Wait for any ongoing transmission then flush UART port
    while(HAL_ESP_UARTBusy());
    _FlushUART();
    while(HAL_ESP_UARTBusy());
#ifdef __DEBUG_SESSION__
    UARTprintf("Sending: %s \n", txBuffer);
#endif
    //  Send char-by-char until reaching end of command
    while (*(txBuffer + txLen) != '\0')
    {
        HAL_ESP_SendChar(*(txBuffer + txLen));
        txLen++;
    }
    //  ESP messages terminated by \r\n
    HAL_ESP_SendChar('\r');
    HAL_ESP_SendChar('\n');

    //  Start listening for reply
    HAL_ESP_IntEnable(true);

    //  If non-blocking mode is not enabled wait for status
    if (!(flags & ESP_NONBLOCKING_MODE))
    {
        HAL_ESP_WDControl(true, timeout);
        while( !(flowControl & ESP_STATUS_OK) &&
                !(flowControl & ESP_STATUS_ERROR) &&
                !(flowControl & flags));

        HAL_ESP_WDControl(false, 0);
        HAL_DelayUS(1000);
        return flowControl;
    }
    else return ESP_NONBLOCKING_MODE;
}

/**
 * Write bytes directly to port (used when sending data of TCP/UDP socket)
 * @param buffer data to send to serial port
 * @param bufLen length of data in buffer
 */
void ESP8266::_RAWPortWrite(const char* buffer, uint16_t bufLen)
{
#ifdef __DEBUG_SESSION__
    UARTprintf("SendingRAWport: %s \n", buffer);
#endif

    for (int i = 0; i < bufLen; i++)
    {
        while(HAL_ESP_UARTBusy());
        HAL_ESP_SendChar(buffer[i]);
    }
}

/**
 * Empty UART's Rx buffer
 */
void ESP8266::_FlushUART()
{
    char temp = 0;
    UNUSED(temp);   //    Suppress unused variable warning
    while (HAL_ESP_CharAvail())
        temp = HAL_ESP_GetChar();
}

/**
 * Convert IP address from string to integer
 * @param ipAddr string containing IP address X.X.X.X where X=0...255
 * @return  uint32_t value of IP passed as string
 */
uint32_t ESP8266::_IPtoInt(char *ipAddr)
{
    uint32_t retVal = 0;
    uint8_t it = 0;
    uint8_t oct = 3;    //IPV4 has 4 octets, 3 down to 0

    //  Starts at first digit of first octet
    while (isdigit(ipAddr[it]))
    {
        char temp[4];
        uint8_t tempIt = 0;

        //  Extract single octet
        while(isdigit(ipAddr[it]))
            temp[ tempIt++ ] = ipAddr[ it++ ];

        it++;   //  Move iterator from dot char
        //  Convert octet to int and push it to return variable
        retVal |= ((uint32_t)stoi((uint8_t*)temp, tempIt)) << (8 * oct--);
    }

    return retVal;
}

/**
 * Get client index in _clients vector based on socket ID
 * @param sockID socket ID
 * @return index in _client vector or 444 if no client matches socket ID
 */
uint16_t ESP8266::_IDtoIndex(uint16_t sockID)
{
    int it;
    for (it = 0; it < _clients.size(); it++)
        if (_clients[it]._id == sockID) return it;
    //  If not found, return any number bigger than 5 as it's maximum number of
    //  clients ESP can have simultaneously
    return 444;
}


/**
 * Interrupt service routine for handling incoming data on UART (Tx)
 */
void UART7RxIntHandler(void)
{
    static char rxBuffer[1024] ;
    static uint16_t rxLen = 0;

    HAL_ESP_ClearInt();     //  Clear interrupt
    HAL_ESP_WDControl(true, 0);    //   Reset watchdog timer

    //  Loop while there are characters in receiving buffer
    while (HAL_ESP_CharAvail())
    {
        char temp = HAL_ESP_GetChar();
        //  Save only characters in valid range (ASCI > 30 or \n, \r)
        if ((temp > 30) || (temp == '\n') || (temp == '\r'))
            rxBuffer[rxLen++] = temp;
        //  Keep in mind buffer size
        rxLen %= 1024;
#ifdef __DEBUG_SESSION__
        if (((temp > 31) && (temp < 126)) || (temp == '\n'))
            UARTprintf("%c", temp);
#endif
    }

    //  All messages terminated by \r\n
    /*
     *  There are 3 occasions when we want to process data in input buffer:
     *  1) We've reached terminator sequence of the message (\r\n)
     *  2) ESP returned '> ' (without terminator) and awaits data
     *  3) Watchdog timer has timed out changing 'flowControl' to "error"
     *      (on timeout WD timer also recalls the interrupt)
     */
    if (( __esp->flowControl == ESP_STATUS_ERROR) && (rxLen > 1))
    {
        rxBuffer[rxLen++] = '\r';
        rxBuffer[rxLen++] = '\n';
    }

    //if ((rxLen > 2) || ( __esp->flowControl == ESP_STATUS_ERROR))
    if (((rxBuffer[rxLen-2] == '\r') && (rxBuffer[rxLen-1] == '\n'))
      || ((rxBuffer[rxLen-2] == '>') && (rxBuffer[rxLen-1] == ' ' ))
      || ( __esp->flowControl == ESP_STATUS_ERROR) )
    {
        HAL_ESP_WDControl(false, 0);    //   Stop watchdog timer
        //  Parse data in receiving buffer
        if (__esp->flowControl == ESP_STATUS_ERROR)
            __esp->flowControl |= __esp->ParseResponse(rxBuffer, rxLen);
        else
            __esp->flowControl = __esp->ParseResponse(rxBuffer, rxLen);
        //  If some data came from one of opened TCP sockets receive it and
        //  pass it to a user-defined function for further processing
        if ((__esp->custHook != 0) && (__esp->flowControl & ESP_STATUS_IPD))
        {
            for (uint8_t i = 0; i < __esp->_clients.size(); i++)
                if (__esp->GetClientByIndex(i)->Ready())
                {
#if defined(__USE_TASK_SCHEDULER__)
            //  If using task scheduler, schedule receiving outside this ISR
                    volatile _taskEntry tE(ESP_UID, ESP_T_RECVSOCK, 0);
                    tE.AddArg(&__esp->GetClientByIndex(i)->_id, 1);
                    __taskSch->SyncTask(tE);
#else
                    //  If no task scheduler do everything in here
                    char resp[128];
                    uint16_t respLen = 0;
                    memset(resp, 0, 128);
                    __esp->GetClientByIndex(i)->Receive(resp, &respLen);
                    __esp->custHook(i, (uint8_t*)resp, &respLen);
#endif
                }
            }
        //  If ESP is running in server mode or status hasn't been OK/ERROR
        //  (because after them ESP sends no more messages) then continue
        //  listening on serial port for new messages
        if (((__esp->flowControl & ESP_STATUS_OK) ||
             (__esp->flowControl & ESP_STATUS_ERROR))
            && !__esp->ServerOpened())
            HAL_ESP_IntEnable(false);

        //  Reset receiving buffer and its size
        memset(rxBuffer, '\0', sizeof(rxBuffer));
        rxLen = 0;
    }
}


