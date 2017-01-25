/**
 * taskScheduler.c
 *
 *  Created on: 30. 7. 2016.
 *      Author: Vedran
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

#include "taskScheduler.h"
#include "tm4c1294_hal.h"


/**
 * @brief Callback vector for all available kernel modules
 * Once a new kernel module is initialized it has a possibility to register its
 * service in task scheduler by providing designated callback function to be
 * called when requesting a service, and memory space for arguments to be
 * transfered to module when requesting a service
 */
static volatile struct _callBackEntry *__callbackVector[10];
/**
 * Register services for a kernel modules into a callback vector
 * @param arg structure with parameters for callback action
 * @param uid Unique identifier of kernel module
 */
void TS_RegCallback(struct _callBackEntry *arg, uint8_t uid)
{
    __callbackVector[uid] = arg;
}


/*  Global pointer to last instance of TaskScheduler object */
volatile TaskScheduler* __taskSch;


/************************************************************
 ************************************************************
 ******		_taskEntry class member functions		*********
 ************************************************************
 ***********************************************************/

_taskEntry::_taskEntry() : _task(0), _argN(0)
{
	for (uint8_t i = 0; i< 10; i++)
		_args[i] = 0.0f;
}

_taskEntry::_taskEntry(volatile _taskEntry& arg)
{
	*this = arg;
}

void _taskEntry::_init() volatile
{
    _libuid = 0;
	_task = 0;
	_argN = 0;
	for (uint8_t i = 0; i< 10; i++)
		_args[i] = 0.0f;
}

void _taskEntry::AddArg(float arg) volatile
{
    memcpy((void*)(_args + 1 + _argN*4), (void*)&arg, 4);
	_argN += 4;
}

uint16_t _taskEntry::GetTask()
{
	return _task;
}

float _taskEntry::GetArg(uint8_t index)
{
	if (index < _argN) return _args[index];
	else return (-1);
}

uint16_t _taskEntry::GetArgNum()
{
	return _argN;
}

/************************************************************
 ************************************************************
 *****  	TaskScheduler class member functions		*****
 ************************************************************
 ***********************************************************/

///-----------------------------------------------------------------------------
///                      Class constructor & destructor                [PUBLIC]
///-----------------------------------------------------------------------------
TaskScheduler::TaskScheduler() :_taskItB(0), _taskItE(0)
{
	for (uint8_t i = 0; i < TS_MAX_TASKS; i++)
		_taskLog[i]._init();
	__taskSch = this;

	HAL_TS_InitSysTick(100, TSSyncCallback);
	HAL_TS_StartSysTick();
}

TaskScheduler::~TaskScheduler()
{
    HAL_TS_StopSysTick();
	__taskSch = 0;
}

///-----------------------------------------------------------------------------
///                      Schedule content manipulation                  [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * @brief	Clear task schedule, remove all enties from ii
 */
void TaskScheduler::Reset() volatile
{
	//	Initialize all variables of the structure to 0
	for (uint8_t i = 0; i < TS_MAX_TASKS; i++)
			_taskLog[i]._init();
	_taskItB = 0;
	_taskItE = 0;
}

/**
 * @brief	Return status of Task scheduler queue
 * @return	true: if there's nothing in queue
 * 		   false: if queue contains data
 */
bool TaskScheduler::IsEmpty() volatile
{
	if (_taskItE > _taskItB)
	    return false;
	else if ((_taskItE != 0) && (_taskItB == _taskItE))
	{
	    Reset();
	    return true;
	}
	else return true;
}

/**
 * @brief	Add new command to task schedule
 * @note	Use AddArgForCurrent for adding arguments connected to command
 *
 */
uint8_t TaskScheduler::PushBackEntry(uint8_t libuid, uint8_t comm) volatile
{
    _taskLog[_taskItE]._libuid = libuid;
	_taskLog[_taskItE]._task = comm;
	_taskItE++;

	return (_taskItE - 1);
}

uint8_t TaskScheduler::PushBackEntrySync(uint8_t libuid, uint8_t comm, uint32_t time) volatile
{
    _taskLog[_taskItE]._libuid = libuid;
    _taskLog[_taskItE]._task = comm;
    _taskLog[_taskItE]._timestamp = time;
    _taskItE++;

    return (_taskItE - 1);
}
/**
 * @brief	Add new argument for last command pushed in scheduler
 * @note	Arguments are added as strings, but saved as floats
 */
void TaskScheduler::AddArgForCurrent(uint8_t* arg, uint8_t argLen) volatile
{
	//_taskLog[_taskItE - 1].AddArg(stof(arg, argLen));
    //  Copy memory directly
    //memcpy((void*)_taskLog[_taskItE - 1]._args, (const void*)arg, argLen);

}

void TaskScheduler::AddStringArg(uint8_t* arg, uint8_t argLen) volatile
{
    //if ((_taskLog[_taskItE - 1]._argN +argLen) <= 50)
    //{

        memset((void*)_taskLog[_taskItE - 1]._args, 0, 50);    //Each memory block is 50B
        memcpy((void*)(_taskLog[_taskItE - 1]._args),
               (void*)arg, argLen);
        _taskLog[_taskItE - 1]._argN = argLen;
        //_taskLog[_taskItE - 1]._args[0] = _taskLog[_taskItE - 1]._argN;
    //}
}

void TaskScheduler::AddNumArg(uint8_t* arg, uint8_t argLen) volatile
{
    _taskLog[_taskItE - 1].AddArg(stof(arg, argLen));
}
/**
 * @brief	Return first element from task queue
 * @return  first element from task queue and delete it (by moving iterators).
 *          If the queue is empty it resets the queue.
 */
volatile _taskEntry& TaskScheduler::PopFront() volatile
{
	_taskItB++;

	return _taskLog[_taskItB - 1];
}

/**
 * @brief	Return task entry at given index
 * @return		nothing: if index is outside of boundaries
 *	  _taskEntry object: if index is valid
 */
volatile _taskEntry& TaskScheduler::At(uint16_t index) volatile
{
	if ((index+_taskItB) < _taskItE)
		return _taskLog[index+_taskItB];
	else
		return _taskLog[0];
}
volatile _taskEntry& TaskScheduler::PeekFront() volatile
{
    return _taskLog[_taskItB];
}
//#################################################Schedule content manipulation
//******************************************************************************
static volatile uint64_t __msSinceStartup = 0;

void TSSyncCallback(void)
{
    __msSinceStartup += HAL_TS_GetTimeStepMS();
    //GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0xFF);
}

void TS_GlobalCheck(void)
{
    if (!__taskSch->IsEmpty())
        while((__taskSch->PeekFront()._timestamp <= __msSinceStartup) &&
              (!__taskSch->IsEmpty()))
        {
            /// Take out first entry to process it
            _taskEntry tE = __taskSch->PopFront();

            /// Check if callback exists
            if ((__callbackVector + tE._libuid) == 0)
                return;

            /// Transfer data into kernel memory space
            (__callbackVector[tE._libuid]->serviceID) = tE._task;
            uint8_t i;
            //Num of args +1 as argN doesn't account for args[0]
            __callbackVector[tE._libuid]->args[0] = tE._argN;
            for (i = 1; i <= (tE._argN); i++)
                __callbackVector[tE._libuid]->args[i] = tE._args[i-1];
            /// Call to kernel module
            __callbackVector[tE._libuid]->callBackFunc();

        }
}
