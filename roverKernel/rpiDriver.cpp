/*
 * rpiDriver.c
 *
 *  Created on: 29. 5. 2016.
 *      Author: Vedran
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "rpiDriver.h"

/*		Tasks lookup table		*/
char taskLookup[][32] = {{ "scan" },
						 { "engnorm" },
						 { "engarc" },
						 { "engperc" }};

/*		Communication settings	 	*/
const char myPrefix[] = {"#TM4C:"};
const char parserPrefix[] = {"#C1905:"};
const char ending[] = {"\r\n"};

RPIRover* __rpiRov;


RPIRover::RPIRover(float wheelD, float wheelS, float vehSiz, float encRes)
	:	_uhw(), _ed(wheelD, wheelS, vehSiz, encRes), _ts(), _rd()
{
	_uhw.AddHook(UARTHook);
	__rpiRov = this;
}

RPIRover::~RPIRover()
{
	__rpiRov = 0;
}

int8_t RPIRover::InitHW()
{
    _dbg.InitHW();
	_uhw.InitHW();

	_ed.InitHW();
	_rd.InitHW();

	_uhw.ConnectAP("UK-224", "hQfmKeUX");
	_uhw.StartTCPServer(27541);
	_uhw.TCPListen(true);


	_dbg.Send("%sinitialization,done%s", myPrefix, ending);
	//_ed.StartEngines(DIRECTION_FORWARD, 10, true);

	return STATUS_OK;
}

void RPIRover::SendErr(char* errMsg)
{
    _dbg.Send("%serror,%s%s",myPrefix, errMsg, ending);
    _uhw.Send("%serror,%s%s",myPrefix, errMsg, ending);
}


/**
 * Extract commands from strings sent by RPi
 * 	Message format= #%deviceID%:command,arg1,arg2,...\r
 */
void RPIRover::ParseRpiCommand(uint8_t *buffer, uint16_t bufLen) volatile
{
	uint16_t startPos = 3,	//	Expected starting position of a command
			 i, it = 0;		//	Message iterator
	uint8_t command[20];	//	Holder for extracted command

	memset(command, 0, 20);
	/**
	 * Each message from RPi starts with "#RPI:", loop through message to find
	 * 		starting condition
	 */
	if (strstr((char*)buffer, parserPrefix) == NULL) return;
	startPos = sizeof(parserPrefix) - 1;	//-1 to compensate for \0 ending char

	/**
	 * Move start position to next char after ':' for next step to extract the
	 * command. Also check if the iterator is outside message length and notify
	 * other side about bad command
	 */
	if (startPos >= bufLen)
	{
		//SendErr("badCommand");
		return;
	}

	//	Put message iterator to beginning of actual message
	it = startPos;

	//	Word following prefix is command to execute
	while (isalpha(buffer[it]))
		command[it - startPos] = buffer[it++];


	//	Find the command in the lookup table and save it to the task queue
	//TODO: Change 0 to the UID of kernel module
	for (i = 1; i <= NUM_OF_TASKS; i++)
		if (strstr((char*)command, taskLookup[i - 1]) != NULL)
			_ts.SyncTask(0, i,0);

	//	If there's no arguments passed finish parsing
	if (buffer[it] != ',') return;

	it++;	//	Move from ',' sign to the first char of argument

	//	Loop until reaching the end of message
	while ((buffer[it] != '\r') && (it < bufLen))
	{
		uint8_t arg[10], argLen = 0;
		memset(arg, 0, 10);

		//	Extract each argument as string and save it
		while (isdigit(buffer[it]) || (buffer[it] == '.') || (buffer[it] == '-'))
			arg[argLen++] = buffer[it++];

		_ts.AddArgs(arg, argLen);
		it++;		//Move from ',' sign to the first char of argument
	}
}

/*		Callback function to process received tasks		*/
int8_t RPIRover::RPICallback()
{
	//Variables for radar scan
	uint8_t sweep[180];
	uint16_t len = 0, i;
	char tempBuf[128];


	return STATUS_OK;
}

void UARTHook(uint8_t, uint8_t* txBuf, uint16_t* txBufLen)
{
	//  Parse the message and reset buffers if end-condition has been met
    if ((*txBufLen) < 2) return;
    if ((txBuf[(*txBufLen)-2] == '\r') && (txBuf[(*txBufLen)-1] == '\n'))
	{
		//Extract relevant data from message
		__rpiRov->ParseRpiCommand(txBuf, *txBufLen);
		//Reset buffer for next message
		memset(txBuf, 0, *txBufLen);
		*txBufLen = 0;
	}
}
