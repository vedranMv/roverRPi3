/**
 *	esp8266.h
 *
 *  Created on: 1. 8. 2016.
 *      Author: Vedran Mikov
 *
 *  @version 1.1.4
 *	@note	Current functionality:
 *      -Connect/disconnect from AP, get acquired IP as string/int
 *	    -Start TCP server and allow multiple connections, keep track of
 *	      connected clients
 *	    -Parse ESP replies and get status of commands
 *      -Parse incoming data from a TCP socket and allow for hooking user routines
 *        to manipulate received socket data
 *      -Send data over TCP socket to a client connected to the TCP server of ESP
 */

/*************TODO block***********************************
 ****+Implement TCP write, reuse socket ID from TCP server
 **********************************************************/

#ifndef ESP8266_H_
#define ESP8266_H_

#include "tm4c1294_hal.h"
#include "taskScheduler.h"
#include <stdio.h>
#include <vector>

#define __DEBUG_SESSION__


/*		Communication settings	 	*/
#define ESP_DEF_BAUD			1000000

/*		ESP8266 error codes		*/
#define ESP_STATUS_LENGTH		13
#define ESP_NO_STATUS			0
#define ESP_STATUS_OK			1<<0
#define ESP_STATUS_BUSY			1<<1
#define ESP_STATUS_ERROR		1<<2
#define ESP_NONBLOCKING_MODE	1<<3
#define ESP_STATUS_CONNECTED	1<<4
#define ESP_STATUS_DISCN        1<<5
#define ESP_STATUS_READY		1<<6
#define ESP_STATUS_SOCKOPEN     1<<7
#define ESP_STATUS_SOCKCLOSE	1<<8
#define ESP_STATUS_RECV			1<<9
#define ESP_STATUS_FAIL			1<<10
#define ESP_STATUS_SENDOK		1<<11
#define ESP_RESPOND_SUCC		1<<12
#define ESP_NORESPONSE          1<<13
#define ESP_STATUS_IPD          1<<14

#define FLAG_DATAPUSH		1<<16

extern uint32_t g_ui32SysClock;

class ESP8266;
class _espClient;

typedef std::vector<_espClient> espCli;


class ESP8266
{
        friend class _espClient;
        friend void _ESP_KernelCallback(void);
	public:
		ESP8266();
		~ESP8266();

		uint32_t    InitHW(int32_t baud = ESP_DEF_BAUD);
		uint32_t    ConnectAP(char* APname, char* APpass);
		bool        IsConnected();
		uint32_t    MyIP();
		uint32_t    DisconnectAP();

		uint32_t    StartTCPServer(uint16_t port);
		_espClient* GetClientByIndex(uint16_t index);
		_espClient* GetClientBySockID(uint16_t id);
		void        TCPListen(bool enable);
		bool        ServerOpened();
		uint32_t    StopTCPServer();

		void 		Enable(bool enable);
		bool		IsEnabled();

		uint32_t    Send2(char* arg);
		uint32_t    Send(const char* arg, ...) { return ESP_NO_STATUS; }
		uint32_t    Execute(uint32_t flags, const char* arg, ...);

		void		AddHook(void((*custHook)(uint8_t*, uint16_t*)));
		uint32_t 	ParseResponse(char* rxBuffer, uint16_t rxLen);

		void	((*custHook)(uint8_t*, uint16_t*));  ///< Hook to user routine
		volatile uint32_t	flowControl;

	protected:
		bool        _InStatus(const uint32_t status, const uint32_t flag);
		uint32_t	_SendRAW(const char* txBuffer, uint32_t flags = 0);
		void        _RAWPortWrite(const char* buffer, uint16_t bufLen);
		void	    _FlushUART();
		uint32_t    _IPtoInt(char *ipAddr);
		uint16_t    _IDtoIndex(uint16_t sockID);

		uint32_t    _ipAddress;
		char        _ipStr[16];

		//  TCP server-related variables
		uint16_t    _tcpServPort;
		bool        _servOpen;

		//  Vector of opened clients
		espCli      _clients;
		_callBackEntry _espSer;
};

class _espClient
{
    friend class ESP8266;
    public:
        _espClient()
            : _parent(0), _id(0) ,_alive(false)  { _Clear(); }
        _espClient(uint8_t id, ESP8266 *par)
            : _parent(par), _id(id), _alive(true) { _Clear(); }
        _espClient(const _espClient &arg)
            : _parent(arg._parent), _id(arg._id), _alive(arg._alive) { _Clear(); }

        void operator= (const _espClient &arg)
        {
            _parent = arg._parent;
            _id = arg._id;
            _alive = arg._alive;
            _respRdy = arg._respRdy;
            for (int i = 0; i < sizeof(_respBody); i++)
                _respBody[i] = arg._respBody[i];
        }

        uint32_t  SendTCP(char *buffer)
        {
            uint16_t bufLen = 0;
            char tempBuf[512];

            while (buffer[bufLen++] != '\0');   //Find length of string
            //  Initiate transmission from
            snprintf(tempBuf, sizeof(tempBuf), "AT+CIPSEND=0,%d\0", bufLen);
            if (_parent->_SendRAW(tempBuf, ESP_STATUS_RECV))
            {
               _parent->flowControl = ESP_NO_STATUS;
               if (!_parent->_servOpen) HAL_ESP_IntEnable(true);
                _parent->_RAWPortWrite(buffer, bufLen);
                while (_parent->flowControl == ESP_NO_STATUS);
            }

            return _parent->flowControl;
        }
        uint32_t Receive(char *buffer, uint16_t *bufferLen)
        {
            (*bufferLen) = 0;
            //  Check if there's new data received
            if (_respRdy)
            {
                //  Fill argument buffer
                while(_respBody[(*bufferLen)] != 0)
                {
                    buffer[(*bufferLen)] = _respBody[(*bufferLen)];
                    (*bufferLen)++;
                }

                //  Empty internal buffer - memset doesn't work on volatile
                _Clear();

                return ESP_NO_STATUS;
            }
            else return ESP_NORESPONSE;
        }
        uint32_t    Close()
        {
            return _parent->Execute(ESP_STATUS_OK, "AT+CIPCLOSE=%d\0", _id);
        }

    private:
        void _Clear()
        {
            for (int i = 0; i < sizeof(_respBody); i++)
                _respBody[i] = 0;
            _respRdy = false;
        }

        ESP8266         *_parent;
        uint8_t         _id;
        volatile bool   _alive;
        volatile bool   _respRdy;
        volatile char   _respBody[128];
};

extern ESP8266* __esp;
/*
 * Function for receiving and processing incomming data - no need to call them
 */
extern "C" void UART7RxIntHandler(void);

#endif /* ESP8266_H_ */
