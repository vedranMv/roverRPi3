/**
 *	uartHW.h
 *
 *  Created on: 30. 7. 2016.
 *      Author: Vedran Mikov
 *
 */
#ifndef UARTHW_H_
#define UARTHW_H_

/*		Communication settings	 	*/
#define COMM_BAUD	115200
#define TX_BUF_LEN	512

/**
 * Interface to a UART-to-USB port, used for debugging
 */
class SerialPort
{
	public:
        static SerialPort& GetI();
        static SerialPort* GetP();

		int8_t	InitHW();
		void	Send(const char* arg, ...);
		void	AddHook(void((*custHook)(uint8_t*, uint16_t*)));

		void	((*custHook)(uint8_t*, uint16_t*));  // Hook to user routine
	protected:
		SerialPort();
        ~SerialPort();

};

/*
 * Function for receiving and processing incomming data - no need to call them
 */

extern "C"
{
    void UART0RxIntHandler(void);
    extern uint32_t g_ui32SysClock;
}

#endif /* UARTHW_H_ */
