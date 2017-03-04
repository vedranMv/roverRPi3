/**
 *	esp8266.h
 *
 *  Created on: 1. 8. 2016.
 *      Author: Vedran Mikov
 *
 *  ESP8266 WiFi module communication library
 *  @version 1.3.1
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
 *  V1.2.1 - 26.1.2017
 *  +Implemented watchdog(WD) timer with variable timeout to handle any blockage
 *      in communication (ESP sometimes not consistent with sending msg terminator)
 *  +Establish a connection to TCP server (uses existing _espClient class)
 *  V1.3 - 31.1.2017
 *  +Integration of library with task scheduler
 *  V1.3.1 - 6.2.2017
 *  +Modified to support Task scheduler v2.3
 *  TODO:Add interface to send UDP packet
 */
#include "roverKernel/hwconfig.h"

//  Compile following section only if hwconfig.h says to include this module
#if !defined(ESP8266_H_) && defined(__HAL_USE_ESP8266__)
#define ESP8266_H_

//  Define class prototype
class ESP8266;
//  Shared buffer for ESP library to assemble text requests in
extern char _commBuf[512];

//  Include client library
#include "espClient.h"

//  Enable debug information printed on serial port
//#define __DEBUG_SESSION__

//  Enable integration of this library with task scheduler but only if task
//  scheduler is being compiled into this project
#if defined(__HAL_USE_TASKSCH__)
#define __USE_TASK_SCHEDULER__
#endif  /* __HAL_USE_TASKSCH__ */

//  Check if this library is set to use task scheduler
#if defined(__USE_TASK_SCHEDULER__)
    #include "roverKernel/taskScheduler/taskScheduler.h"
    //  Unique identifier of this module as registered in task scheduler
    #define ESP_UID         0
    //  Definitions of ServiceID for service offered by this module
    #define ESP_T_TCPSERV   0   //  Start/stop TCP server
    #define ESP_T_CONNTCP   1   //  Open TCP connection to server
    #define ESP_T_SENDTCP   2   //  Send data through socket with specific ID
    #define ESP_T_RECVSOCK  3   //  Receive data from specific socket ID
    #define ESP_T_CLOSETCP  4   //  Close socket with specific ID

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

/**
 * ESP8266 class definition
 */
class ESP8266
{
        /// Functions & classes needing direct access to all members
        friend class    _espClient;
        friend void     UART7RxIntHandler(void);
        friend void     _ESP_KernelCallback(void);
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

		void		AddHook(void((*funPoint)(uint8_t, uint8_t*, uint16_t*)));
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
		_kernelEntry _espKer;
#endif
};

//  Global pointer to FIRST created instance of ESP module
extern ESP8266* __esp;
/*
 *  UART interrupt handle - used to receive incoming data from ESP
 */
extern void UART7RxIntHandler(void);

#endif /* ESP8266_H_ */
