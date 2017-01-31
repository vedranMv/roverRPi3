/**
 *	esp8266.h
 *
 *  Created on: 1. 8. 2016.
 *      Author: Vedran Mikov
 *
 *  ESP8266 WiFi module communication library
 *  @version 1.1.5
 *  V1.1.4
 *  +Connect/disconnect from AP, get acquired IP as string/int
 *	+Start TCP server and allow multiple connections, keep track of
 *	    connected clients
 *  +Parse ESP replies and get status of commands
 *  +Parse incoming data from a TCP socket and allow for hooking user routines
 *      to manipulate received socket data
 *  +Send data over TCP socket to a client connected to the TCP server of ESP
 *  V1.2 - 25.1.2017
 *  +When in server mode can reply back to clients over TCP socket
 *  +On initialization library registers its services as a kernel module in task
 *  scheduler
 *  V1.2.1 26.1.2017
 *  +Implemented watchdog(WD) timer with variable timeout to handle any blockage
 *      in communication (ESP sometimes not consistent with sending msg terminator)
 *  +Establish a connection to TCP server (uses existing _espClient class)
 *  Add interface to send UDP packet
 */

#ifndef ESP8266_H_
#define ESP8266_H_

#include <vector>

//  Enable debug information printed on serial port
//#define __DEBUG_SESSION__
//  Enable integration of this library with task scheduler
#define __USE_TASK_SCHEDULER__

#if defined(__USE_TASK_SCHEDULER__)
    #include "taskScheduler.h"
    //  Unique identifier of this module as registered in task scheduler
    #define ESP_UID         0
    //  Definitions of ServiceID for service offered by this module
    #define ESP_T_TCPSERV   0   //  Start/stop TCP server
    #define ESP_T_CONNTCP   1   //  Open TCP connection to server
    #define ESP_T_SENDTCP   2   //  Send data through socket with specific ID

#endif


/*		Communication settings	 	*/
#define ESP_DEF_BAUD			1000000

/*		ESP8266 error codes		*/
#define ESP_STATUS_LENGTH		13
#define ESP_NO_STATUS			0
#define ESP_STATUS_OK			1<<0
#define ESP_STATUS_BUSY			1<<1
#define ESP_RESPOND_SUCC		1<<2
#define ESP_NONBLOCKING_MODE	1<<3
#define ESP_STATUS_CONNECTED	1<<4
#define ESP_STATUS_DISCN        1<<5
#define ESP_STATUS_READY		1<<6
#define ESP_STATUS_SOCKOPEN     1<<7
#define ESP_STATUS_SOCKCLOSE	1<<8
#define ESP_STATUS_RECV			1<<9
#define ESP_STATUS_FAIL			1<<10
#define ESP_STATUS_SENDOK		1<<11
#define ESP_STATUS_ERROR		1<<12
#define ESP_NORESPONSE          1<<13
#define ESP_STATUS_IPD          1<<14
#define ESP_GOT_IP              1<<15


/// Clss prototypes definition
class ESP8266;
class _espClient;
/// Short annotation for vector of TCP clients
typedef std::vector<_espClient> espCli;

/**
 * _espClient class - wrapper for TCP client connected to ESP server
 */
class _espClient
{
    friend class ESP8266;
    friend void UART7RxIntHandler(void);
    public:
        _espClient();
        _espClient(uint8_t id, ESP8266 *par);
        _espClient(const _espClient &arg);

        void        operator= (const _espClient &arg);

        uint32_t    SendTCP(char *buffer);
        uint32_t    Receive(char *buffer, uint16_t *bufferLen);
        uint32_t    Close();

        //  Keep socket alive (don't terminate it after first round of communication)
        volatile bool   KeepAlive;

    private:
        void        _Clear();

        //  Pointer to a parent device of of this client
        ESP8266         *_parent;
        //  Socket ID of this client, as returned by ESP
        uint8_t         _id;
        //  Specifies whether the socket is alive
        volatile bool   _alive;
        //  Specifies whether there's a response from this client ready to read
        volatile bool   _respRdy;
        //  Buffer for data received on this socket
        volatile char   _respBody[1024];
};

/**
 * ESP8266 class definition
 */
class ESP8266
{
        /// Functions & classes needing direct access to all members
        friend class _espClient;
        friend void UART7RxIntHandler(void);
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

		uint32_t    OpenTCPSock(char *ipAddr, uint16_t port, bool keepAlive=true);
		bool        ValidSocket(uint16_t id);

		void 		Enable(bool enable);
		bool		IsEnabled();

		uint32_t    Send2(char* arg);
		uint32_t    Send(const char* arg, ...) { return ESP_NO_STATUS; }

		void		AddHook(void((*custHook)(uint8_t, uint8_t*, uint16_t*)));
		uint32_t 	ParseResponse(char* rxBuffer, uint16_t rxLen);

		//  Hook to user routine called when data from socket is received
		void	((*custHook)(uint8_t, uint8_t*, uint16_t*));
		//  Status variable for error codes returned by ESP - bi
		volatile uint32_t	flowControl;

	protected:
		bool        _InStatus(const uint32_t status, const uint32_t flag);
		uint32_t	_SendRAW(const char* txBuffer, uint32_t flags = 0,
		                     uint32_t timeout = 1000);
		void        _RAWPortWrite(const char* buffer, uint16_t bufLen);
		void	    _FlushUART();
		uint32_t    _IPtoInt(char *ipAddr);
		uint16_t    _IDtoIndex(uint16_t sockID);

		//  IP address in decimal and string format
		uint32_t    _ipAddress;
		char        _ipStr[16];

		//  TCP server port
		uint16_t    _tcpServPort;
		//  Specifies whether the TCP server is currently running
		bool        _servOpen;

		//  Vector clients currently connected to ESP (when in TCP-server mode)
		//  or connections initiated by the ESP
		espCli      _clients;

		//  Interface with task scheduler - provides memory space and function
		//  to call in order for task scheduler to request service from this module
#if defined(__USE_TASK_SCHEDULER__)
		_callBackEntry _espSer;
#endif
};

//  Global pointer to FIRST created instance of ESP module
extern ESP8266* __esp;
/*
 *  UART interrupt handle - used to receive incoming data from ESP
 */
extern void UART7RxIntHandler(void);

#endif /* ESP8266_H_ */
