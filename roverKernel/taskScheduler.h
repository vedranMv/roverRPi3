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
 *  Implement UTC clock ability, if at some point program finds out what the
 *  actual time is it can save it and maintain real UTC time reference
 *
 ****Hardware dependencies:
 *  SysTick timer & interrupt
 */

#ifndef TASKSCHEDULER_H_
#define TASKSCHEDULER_H_

#include "myLib.h"

/// Max number of task in TaskSchedule queue
#define TS_MAX_TASKS	10
#define TS_TASK_MEMORY  50


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
		_taskEntry(volatile _taskEntry& arg);
		_taskEntry(uint8_t task, uint32_t utcTime): _task(task), _timestamp(utcTime){};

		void 		AddArg(float arg) volatile;
		uint16_t 	GetTask();
		float		GetArg(uint8_t index);
		uint16_t	GetArgNum();

		void operator= (volatile _taskEntry& arg) volatile
		{
		    _libuid = arg._libuid;
			_task = arg._task;
			_argN = arg._argN;
			_timestamp = arg._timestamp;

			for (uint8_t i = 0; i < sizeof(_args); i++)
				_args[i] = arg._args[i];
		}

	protected:
		         void       _init() volatile;
		//  Unique ientifier for library to request service from
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
 * @note Task and it arguments ar added separately. First add new task and then
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

		void 				 Reset() volatile;
		bool				 IsEmpty() volatile;
		uint8_t 			 PushBackEntry(uint8_t libuid, uint8_t comm) volatile;
		uint8_t              PushBackEntrySync(uint8_t libuid, uint8_t comm,
		                                       uint32_t time) volatile;
		void 				 AddArgForCurrent(uint8_t* arg,
											  uint8_t argLen) volatile;
		void                 AddStringArg(uint8_t* arg, uint8_t argLen) volatile;
		void                 AddNumArg(uint8_t* arg, uint8_t argLen) volatile;
		volatile _taskEntry& PopFront() volatile;
		volatile _taskEntry& PeekFront() volatile;
		volatile _taskEntry& At(uint16_t index) volatile;

	private:
		//  Queue of tasks to be executed
		volatile _taskEntry	_taskLog[TS_MAX_TASKS];
		//  Iterators for task queue (B-begin, E-end)
		volatile int8_t		_taskItB,
							_taskItE;
};

/*	Global pointer to last instance of TaskScheduler object	*/
extern volatile TaskScheduler* __taskSch;
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
 * @note IMPORTANT! 1st byte of args ALWAYS contains number of following data bytes
 *  remember to use +1 offset in memory when ding memcpy on args array
 */
struct _callBackEntry
{
    void((*callBackFunc)(void));    // Pointer to callback function
    uint8_t serviceID;              // Requested service
    uint8_t args[TS_TASK_MEMORY];   // Arguments for service execution
    int8_t  retVal;                 // (Optinal) Return variable of service exec
};

extern void TS_RegCallback(struct _callBackEntry *arg, uint8_t uid);


#endif /* TASKSCHEDULER_H_ */
