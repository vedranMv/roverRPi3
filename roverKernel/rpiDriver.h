/*
 * rpiDriver.h
 *
 *  Created on: 30. 5. 2016.
 *      Author: Vedran
 *
 */

#ifndef RPIDRIVER_H_
#define RPIDRIVER_H_

#include "myLib.h"
#include "uartHW.h"
#include "engines.h"
#include "radarGP2.h"
#include "taskScheduler.h"
#include "esp8266.h"

/*		Tasks		*/
#define NUM_OF_TASKS	4
#define T_RADARSCAN		1
#define T_ENGNORM		2
#define T_ENGARC		3
#define T_ENGPERC		4


class RPIRover
{
	public:
		RPIRover(float wheelD, float wheelS, float vehSiz, float encRes);
		~RPIRover();

		int8_t	InitHW();
		int8_t 	RPICallback();
		void 	ParseRpiCommand(uint8_t *buffer, uint16_t bufLen) volatile;

		void 	SendErr(char* errMsg);
		void    Resp()
		{
		    char buffer[128];
		    uint16_t len = 0;
		    memset(buffer,0,128);
		    if (_uhw.GetClientBySockID(0)->Receive(buffer, &len) != ESP_NORESPONSE)
		        UARTprintf("%s", buffer);
		}
		ESP8266*    GetComm()
		{
		    return &_uhw;
		}

	private:
		ESP8266         _uhw;
		UartHW          _dbg;
		EngineData 		_ed;
		TaskScheduler 	_ts;
		RadarData		_rd;

};

extern RPIRover* __rpiRov;

extern "C" void UARTHook(uint8_t* txBuf, uint16_t* txBufLen);

#endif /* RPIDRIVER_H_ */
