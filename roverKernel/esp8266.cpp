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

#include "myLib.h"
#include "esp8266.h"
#include "utils/uartstdio.h"
#include "taskScheduler.h"

void _ESP_KernelCallback(void);

/*      Lookup table for statuses returned by ESP8266       */
/*const char status_table[][20]={ {"OK"}, {"BUSY"}, {"ERROR"}, {"NONBLOCKING"},
                                {"CONNECTED"}, {"DISCONNECTED"}, {"READY"},
                                {"SOCK_OPEN"}, {"SOCK_CLOSED"}, {"RECEIVE"},
                                {"FAILED"}, {"SEND OK"}, {"SUCCESS"}};*/

//  Dummy instance used for return value when no available client
_espClient dummy;

//  Pointer to last created instance of ESP8266
ESP8266* __esp;

//  Dummy function te be called to suppress "Unused variable" warnings
void UNUSED(int32_t arg) { }


///-----------------------------------------------------------------------------
///                      Class constructor & destructor                [PUBLIC]
///-----------------------------------------------------------------------------

ESP8266::ESP8266()    :    custHook(0), flowControl(ESP_NO_STATUS)
{
    __esp = this;
   // _clients.resize(5); //  ESP allows for max of 5 clients at the time
}

ESP8266::~ESP8266()
{
    Enable(false);
    __esp = 0;
}


/**
 * Send a command passed as argument to ESP and wait for status given in flags mask
 * @param flags mask of ESP return statuses to wait for in response
 * @param arg vararg combination of arguments used to construct command
 * @return error code, depending on the outcome
 */
uint32_t ESP8266::Execute(uint32_t flags, const char* arg, ...)
{
    va_list vaArgP;
    char buffer[512];
    uint32_t retVal = ESP_NO_STATUS;

    //    Start the varargs processing.
    va_start(vaArgP, arg);

    snprintf(buffer, sizeof(buffer), arg, vaArgP);
    snprintf(buffer, sizeof(buffer), "%s\0", buffer);
    retVal = _SendRAW(buffer);

    //    We're finished with the varargs now.
    va_end(vaArgP);

    return retVal;
}

///-----------------------------------------------------------------------------
///         Public functions used for configuring ESP8266               [PUBLIC]
///-----------------------------------------------------------------------------


/**
 * Initialize UART port used in communication with Raspberry Pi
 * @param baud baudrate used in serial communication between ESP and hardware
 * @return error code, depending on the outcome
 */
uint32_t ESP8266::InitHW(int32_t baud)
{
    HAL_ESP_InitPort(baud);
    HAL_ESP_RegisterIntHandler(UART7RxIntHandler);

    //    Turn ESP8266 chip ON
    Enable(true);

    //  Send test command(AT) then turn off echoing of commands(ATE0)
    _SendRAW("AT\0");
    _SendRAW("ATE0\0");

    //  Register module services with task scheduler
    _espSer.callBackFunc = _ESP_KernelCallback;
    TS_RegCallback(&_espSer, 0);


    return ESP_STATUS_OK;
}

/**
 * Enable/disable ESP8266 chip (by controlling CH_PD pin)
 * @param enable new state of chip to set
 */
void ESP8266::Enable(bool enable)
{
    HAL_ESP_HWEnable(enable);
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
 * @param funPoint pointer to void function with 2 arguments: uint8_t*, uint16_t*
 */
void ESP8266::AddHook(void((*funPoint)(uint8_t*, uint16_t*)))
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
    char command[100];

    //  Set ESP in client mode
    retVal = _SendRAW("AT+CWMODE_CUR=1\0");
    if (!_InStatus(retVal, ESP_STATUS_OK)) return retVal;

    //  Assemble command & send it
    snprintf(command, 100, "AT+CWJAP_CUR=\"%s\",\"%s\"\0", APname, APpass);
    retVal = _SendRAW(command);
    if (!_InStatus(retVal, ESP_STATUS_OK)) return retVal;

    //  Acquire IP address and save it locally
    MyIP();

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
    //BUG: When using debug mode (printing all characters incoming on serial) \
        this line will hang as no OK status will be received from ESP
    if (_ipAddress == 0) _SendRAW("AT+CIPSTA_CUR?\0", ESP_GOT_IP);

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
    char command[50];

    //  Allow for multiple connections, in case of error return
    retVal = _SendRAW("AT+CIPMUX=1\0");
    if (!_InStatus(retVal, ESP_STATUS_OK)) return retVal;

    //  Start TCP server, in case of error return
    snprintf(command, sizeof(command), "AT+CIPSERVER=1,%d\0", port);
    retVal = _SendRAW(command);
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
 * Sens a message to TCP client on socket ID 0 (if exists)
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
    uint32_t retVal = ESP_NO_STATUS;
    int16_t ipFlag= -1, respFlag = -1, sockOflag = -1, sockCflag = -1;
    char *retTemp;

    if (rxLen < 1)    return retVal;

    if ((retTemp = strstr(rxBuffer,"ip:\"")) != NULL)
        ipFlag = retTemp - rxBuffer + 4;

    if ((retTemp = strstr(rxBuffer,"+IPD,")) != NULL)
    {
        respFlag = retTemp - rxBuffer + 5;
        retVal |= ESP_STATUS_IPD;
    }

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

    if ((retTemp =strstr(rxBuffer,",CONNECT")) != NULL)
    {
        sockOflag = retTemp - rxBuffer - 1; //Sock ID position
        retVal |= ESP_STATUS_SOCKOPEN;
    }
    if ((retTemp =strstr(rxBuffer,",CLOSED")) != NULL)
    {
        sockCflag = retTemp - rxBuffer - 1; //Sock ID position
        retVal |= ESP_STATUS_SOCKCLOSE;
    }

    if (strstr(rxBuffer,"SEND OK") != NULL)
        retVal |= ESP_STATUS_SENDOK;
    if (strstr(rxBuffer,"SUCCESS") != NULL)
        retVal |= ESP_RESPOND_SUCC;
    if (strstr(rxBuffer,">") != NULL)
        retVal |= ESP_STATUS_RECV;

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
    if (respFlag >= 0)
    {
        int i;
        _espClient *cli = &_clients[_IDtoIndex(rxBuffer[respFlag] - 48)];


        i = respFlag;
        //  Doubledot marks beginning of the message; find it
        while(rxBuffer[i++] != ':');
        //  Use raspFlag to mark starting point
        respFlag = i;
        //  Loop until meeting a terminating character
        while(rxBuffer[i] != '\n')
        {
            cli->_respBody[i - respFlag] = rxBuffer[i];
            i++;
        }
        cli->_respBody[i - respFlag] = '\n';
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
 * functon, awaiting reply from ESP. Function returns when status OK or ERROR
 * or any other status passed in flags have been received from ESP.
 * @param txBuffer null-terminated string with command to execute
 * @param flags bitwise OR of ESP_STATUS_* values
 * @return bitwise OR of ESP_STATUS_* returned by the ESP module
 */
uint32_t ESP8266::_SendRAW(const char* txBuffer, uint32_t flags)
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

    //  If non-blockign mode is not enabled wait for status
    if (!(flags & ESP_NONBLOCKING_MODE))
    {
        while( !(flowControl & ESP_STATUS_OK) &&
                !(flowControl & ESP_STATUS_ERROR) &&
                !(flowControl & flags));

        return flowControl;
    }
    else return ESP_NONBLOCKING_MODE;
}

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
    uint8_t oct = 3;    //IPV4 has 4 octets, 3 downto 0

    //  Starts at first digit of first octet
    while (isdigit(ipAddr[it]))
    {
        char temp[4];
        uint8_t tempIt = 0;

        //  Extract single octet
        while(isdigit(ipAddr[it]))
            temp[ tempIt++ ] = ipAddr[ it++ ];

        it++;   //Move iterrator from dot char
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
    return 444;
}

/**
 * Interrupt service routine for handling incoming data on UART (Tx)
 */
void UART7RxIntHandler(void)
{
    static char rxBuffer[1024] ;
    static uint16_t rxLen = 0;

    HAL_ESP_ClearInt();

    while (HAL_ESP_CharAvail())
    {
        char temp = HAL_ESP_GetChar();
        if ((temp > 30) || (temp == '\n') || (temp == '\r'))
            rxBuffer[rxLen++] = temp;

        rxLen %= 1024;
#ifdef __DEBUG_SESSION__
        if (((temp > 31) && (temp < 126))
                || (temp == '\n')) UARTprintf("%c", temp);
#endif
        }


    //  All messages terminated by \r\n
    if (((rxBuffer[rxLen-2] == '\r') && (rxBuffer[rxLen-1] == '\n'))
            || (rxBuffer[rxLen-1] == '>'))
        {
            __esp->flowControl = __esp->ParseResponse(rxBuffer, rxLen);

            if ((__esp->custHook != 0) && (__esp->flowControl & ESP_STATUS_IPD))
            {
                char resp[128];
                uint16_t respLen = 0;
                memset(resp, 0, 128);
                __esp->GetClientBySockID(0)->Receive(resp, &respLen);
                __esp->custHook((uint8_t*)resp, &respLen);
            }

            if (((__esp->flowControl & ESP_STATUS_OK) ||
                 (__esp->flowControl & ESP_STATUS_ERROR))
                && !__esp->ServerOpened())
                HAL_ESP_IntEnable(false);

            memset(rxBuffer, '\0', sizeof(rxBuffer));
            rxLen = 0;
        }
}

void _ESP_KernelCallback(void)
{
    uint8_t msg[20] = {0};

    memcpy((void*)msg, (void*)(__esp->_espSer.args+1), __esp->_espSer.args[0]);
    UARTprintf("My message is: %s   \r\n", msg);
    msg[__esp->_espSer.args[0]] = '\0';
    __esp->GetClientBySockID(__esp->_espSer.serviceID)->SendTCP((char*)msg);
}



