/**
 *	taskScheduler.h
 *
 *  Created on: 30.7. 2016.
 *      Author: Vedran
 *
 *  Task scheduler library
 *  v1.1
 *  +Implementation of queue of tasks with various parameters. Tasks identified
 *      by unique integer number (defined by higher level library)
 *  v2.1 - 22.1.2017
 *  +Added time component to task entries - each task now has a time stamp at
 *      which it needs to be executed
 *  +Implemented SysTick in interrupt mode to count time from startup providing
 *      time reference for performing task at desired point in time from startup
 *  +Added callback registration for all kernel modules to register their services
 *  +Callback functionality from now on implemented so that module first registers
 *      its service by adding entry into the callback vector. Once the task
 *      scheduler requires that service it will transfer necessary memory into
 *      kernel space and call provided callback function for particular module
 *  v2.2 - TODO
 *  Implement UTC clock feature. If at some point program finds out what the
 *  actual time is it can save it and maintain real UTC time reference
 *
 ****Hardware dependencies:
 *  SysTick timer & interrupt
 */

#ifndef TASKSCHEDULER_H_
#define TASKSCHEDULER_H_

#include "myLib.h"
#include <vector>

/// Max number of task in TaskSchedule queue
#define TS_TASK_MEMORY  50
//#define __DEBUG_SESSION__

/**
 * _taksEntry class - object wrapper for tasks handled by TaskScheduler class
 */
class _taskEntry
{
    // Functions & classes needing direct access to all members
	friend class TaskScheduler;
	friend void TSSyncCallback(void);
    friend void TS_GlobalCheck(void);

	public:
		_taskEntry();
		_taskEntry(const _taskEntry& arg);
		_taskEntry(uint8_t uid, uint8_t task, uint32_t utcTime)
		    :_libuid(uid), _task(task), _timestamp(utcTime) {};

		void 		AddArg(float arg);
		void        AddArg(void* arg, uint8_t argLen);
		uint16_t 	GetTask();
		float		GetArg(uint8_t index);//Deprecated
		uint16_t	GetArgNum();

		_taskEntry& operator= (const _taskEntry& arg)
		{
		    _libuid = arg._libuid;
			_task = arg._task;
			_argN = arg._argN;
			_timestamp = arg._timestamp;

			for (uint8_t i = 0; i < sizeof(_args); i++)
				_args[i] = arg._args[i];
			return *this;
		}

	protected:
		         void       _init();
		//  Unique identifier for library to request service from
		volatile uint8_t    _libuid;
		//  Service ID to execute
		volatile uint8_t    _task;
		//  Number of arguments provided when doing service call
		volatile uint8_t    _argN;
		//  Time at which to exec. service (in ms from start-up of task scheduler)
		volatile uint32_t   _timestamp;
		//  Arguments used when calling service
		volatile uint8_t    _args[TS_TASK_MEMORY];
};

/**
 * Task scheduler class implementation
 * @note Task and it arguments are added separately. First add new task and then
 * use some of the 'Add*Arg' function to add argument(s) for that task
 */
class TaskScheduler
{
    // Functions & classes needing direct access to all members
    friend void TSSyncCallback(void);
    friend void TS_GlobalCheck(void);
	public:
		TaskScheduler();
		~TaskScheduler();

		bool    operator()(const _taskEntry& arg1, const _taskEntry& arg2);
		void 				 Reset();
		bool				 IsEmpty();
		uint8_t              SyncTask(uint8_t libuid, uint8_t comm,
		                                       int64_t time);
		uint8_t              SyncTask(_taskEntry te);
		void                 AddStringArg(void* arg, uint8_t argLen);
		void                 AddNumArg(float arg);
		void                 AddNumArg(uint8_t* arg, uint8_t argLen);
		_taskEntry           PopFront();
		_taskEntry&          PeekFront();
		_taskEntry&          At(uint16_t index);

	private:
		//  Queue of tasks to be executed
		//volatile _taskEntry	_taskLog[TS_MAX_TASKS];
		std::vector<_taskEntry> _taskLog;
		//  Iterators for task queue (B-begin, E-end)
		volatile int8_t		_lastIndex;
};

/*	Global pointer to first instance of TaskScheduler object	*/
extern TaskScheduler* __taskSch;

extern void TSSyncCallback(void);
extern void TS_GlobalCheck(void);

/**
 * Callback entry into the Task scheduler from individual kernel module
 * Once initialized, each kernel module registers the services it provides into
 * a vector by inserting CallBackEntry into a global vector (handled by
 * TS_RegCallback function). CallBackEntry holds: a) Function to be called when
 * someone requests a service from kernel module; b) ServiceID of service to be
 * executed; c)Memory space used for arguments for callback function; d) Return
 * variable of the service execution
 * @note IMPORTANT! 1st byte of args ALWAYS contains number of following data
 *  bytes, remember to use +1 offset in memory when doing memcpy on args array
 */
struct _callBackEntry
{
    void((*callBackFunc)(void));    // Pointer to callback function
    uint8_t serviceID;              // Requested service
    uint8_t args[TS_TASK_MEMORY];   // Arguments for service execution
    int32_t  retVal;                // (Optional) Return variable of service exec
};

extern void TS_RegCallback(struct _callBackEntry *arg, uint8_t uid);

/// Internal time since TaskScheduler startup (in ms)
extern volatile uint64_t __msSinceStartup;

#endif /* TASKSCHEDULER_H_ */
