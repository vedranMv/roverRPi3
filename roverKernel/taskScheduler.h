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
 *
 ****Hardware dependencies:
 *  SysTick timer & interrupt
 */

#ifndef TASKSCHEDULER_H_
#define TASKSCHEDULER_H_

#include "myLib.h"

/// Max number of task in TaskSchedule queue
#define TS_MAX_TASKS	10


/* Add flags to trigger desired functions - add functions in parser in .c file */
class _taskEntry
{
	friend class TaskScheduler;
	friend void TSSyncCallback(void);

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

			for (uint8_t i = 0; i< 10; i++)
				_args[i] = arg._args[i];
		}

	protected:
		void _init() volatile;
		volatile uint8_t    _libuid;
		volatile uint8_t 	_task;
		volatile uint8_t 	_argN;
		volatile uint32_t   _timestamp;
		volatile float 		_args[10];
};

class TaskScheduler
{
    friend void TSSyncCallback(void);
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
		volatile _taskEntry& PopFront() volatile;
		volatile _taskEntry& PeekFront() volatile;
		volatile _taskEntry& At(uint16_t index) volatile;

	private:
		volatile _taskEntry	_taskLog[TS_MAX_TASKS];
		volatile int8_t		_taskItB,
							_taskItE;
};

/*	Global pointer to last instance of TaskScheduler object	*/
extern volatile TaskScheduler* __taskSch;

extern void TSSyncCallback(void);


struct _callBackEntry
{
    void((*callBackFunc)(void));
    uint8_t *serviceID;
    float   *args;
    int8_t  *retVal;
};

extern void TS_RegCallback(struct _callBackEntry arg, uint8_t uid);


#endif /* TASKSCHEDULER_H_ */
