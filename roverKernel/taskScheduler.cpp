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
#include "utils/uartstdio.h"


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


/*
 * Global pointer to FIRST instance of TaskScheduler object
 * (doesn't have to be volatile as this function shouldn't be called from ISR)
 */
TaskScheduler* __taskSch;

/*******************************************************************************
 *******************************************************************************
 *********            _taskEntry class member functions                *********
 *******************************************************************************
 ******************************************************************************/

_taskEntry::_taskEntry() : _libuid(0), _task(0), _argN(0), _timestamp(0)
{
    memset((void*)_args, 0, TS_TASK_MEMORY);
}

_taskEntry::_taskEntry(const _taskEntry& arg)
{
    *this = (const _taskEntry&)arg;
}

void _taskEntry::_init()
{
    _libuid = 0;
	_task = 0;
	_argN = 0;
	_timestamp = 0;
	memset((void*)_args, 0, TS_TASK_MEMORY);
}

void _taskEntry::AddArg(float arg)
{
    memcpy((void*)(_args + 1 + _argN*4), (void*)&arg, 4);
	_argN += 4;
}

void _taskEntry::AddArg(void* arg, uint8_t argLen)
{
    memcpy((void*)(_args+_argN), arg, argLen);
    _argN += argLen;
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

/*******************************************************************************
 *******************************************************************************
 *********  	       TaskScheduler class member functions	    	   *********
 *******************************************************************************
 ******************************************************************************/

///-----------------------------------------------------------------------------
///                      Class constructor & destructor                [PUBLIC]
///-----------------------------------------------------------------------------
TaskScheduler::TaskScheduler()
{
	if (__taskSch == 0) __taskSch = this;

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

bool TaskScheduler::operator()(const _taskEntry& arg1, const _taskEntry& arg2)
{
    if (arg1._timestamp < arg2._timestamp)
        return true;
    return false;
}

/**
 * @brief	Clear task schedule, remove all entries from ii
 */
void TaskScheduler::Reset()
{
    _taskLog.clear();
}

/**
 * @brief	Return status of Task scheduler queue
 * @return	true: if there's nothing in queue
 * 		   false: if queue contains data
 */
bool TaskScheduler::IsEmpty()
{
    return (_taskLog.size() == 0);
}

/**
 * Add task to the back of task queue
 * @param libuid UID of library to call
 * @param comm task ID within the library to execute
 * @param time time stamp at which to execute the task
 * @return index of task in task queue
 */
uint8_t TaskScheduler::SyncTask(uint8_t libuid, uint8_t comm, int64_t time)
{
    if (time < 0)
        time = (uint32_t)(-time) + __msSinceStartup;
    else
        time = (uint32_t)time;

    uint8_t i;
    for (i = 0; i < _taskLog.size(); i++)
        if (_taskLog[i]._timestamp > time)
            break;
    _taskLog.insert(_taskLog.begin()+i, _taskEntry(libuid, comm, time));

    _lastIndex = i;
    return (_taskLog.size()-1);
}

uint8_t TaskScheduler::SyncTask(_taskEntry te)
{
    uint8_t i;
    for (i = 0; i < _taskLog.size(); i++)
        if (_taskLog[i]._timestamp > te._timestamp)
            break;
    _taskLog.insert(_taskLog.begin()+i, te);
    _lastIndex = i;
    return i;
}

void TaskScheduler::AddStringArg(void* arg, uint8_t argLen)
{
    if (_lastIndex < _taskLog.size())
        _taskLog[_lastIndex].AddArg(arg, argLen);
    else
        _lastIndex = 0;
}

void TaskScheduler::AddNumArg(float arg)
{
    if (_lastIndex < _taskLog.size())
        _taskLog[_lastIndex].AddArg(arg);
    else
        _lastIndex = 0;
}

void TaskScheduler::AddNumArg(uint8_t* arg, uint8_t argLen)
{
    if (_lastIndex < _taskLog.size())
        _taskLog[_lastIndex].AddArg(stof(arg, argLen));
    else
        _lastIndex = 0;
}

/**
 * @brief	Return first element from task queue
 * @return  first element from task queue and delete it (by moving iterators).
 *          If the queue is empty it resets the queue.
 */
_taskEntry/*&*/ TaskScheduler::PopFront()
{
    _taskEntry retVal = _taskLog[0];
    _taskLog.erase(_taskLog.begin());
    return retVal;
}

/**
 * @brief	Return task entry at given index
 * @return		nothing: if index is outside of boundaries
 *	  _taskEntry object: if index is valid
 */
_taskEntry& TaskScheduler::At(uint16_t index)
{
    if (index < _taskLog.size())
        return _taskLog[index];
    else
        return *_taskLog.begin();
}
_taskEntry& TaskScheduler::PeekFront()
{
    return _taskLog[0];
}

/*******************************************************************************
 *******************************************************************************
 *********             SysTick callback and time tracking              *********
 *******************************************************************************
 ******************************************************************************/

/// Internal time since TaskScheduler startup (in ms)
volatile uint64_t __msSinceStartup = 0;

/**
 * SysTick interrupt
 * Used to keep internal track of time either as number of milliseconds passed
 * from start-up of task scheduler or acquired UTC time
 */
void TSSyncCallback(void)
{
    __msSinceStartup += HAL_TS_GetTimeStepMS();
}

/**
 * Task scheduler callback routine
 * This routine has to be called in order to execute tasks pushed in task queue
 * and is recently removed from TSSynceCallback because some task might rely on
 * interrupt routines that can't be executed while MCU is within TSSyncCallback
 * function which is a SysTick ISR. (no interrupts while in ISR)
 */
void TS_GlobalCheck(void)
{
    //  Check if there task scheduled to execute
    if (!__taskSch->IsEmpty())
        //  Check if the first task had to be executed already
        while((__taskSch->PeekFront()._timestamp <= __msSinceStartup) &&
              (!__taskSch->IsEmpty()))
        {
            // Take out first entry to process it
            _taskEntry tE;
            tE = __taskSch->PopFront();

            // Check if callback exists
            if ((__callbackVector + tE._libuid) == 0)
                return;

            // Transfer data for task into kernel memory space
            __callbackVector[tE._libuid]->serviceID = tE._task;
            __callbackVector[tE._libuid]->args[0] = tE._argN;
            memcpy( (void*)(__callbackVector[tE._libuid]->args+1),
                    (void*)(tE._args),
                    tE._argN);
#if defined(__DEBUG_SESSION__)
            UARTprintf("Processing %d:%d at %ul ms\n", tE._libuid, tE._task, tE._timestamp);
            UARTprintf("-(%d)> %s\n", tE._argN, tE._args);
#endif
            // Call kernel module to execute task
            __callbackVector[tE._libuid]->callBackFunc();
        }
}
