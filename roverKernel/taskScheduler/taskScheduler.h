/**
 *	taskScheduler.h
 *
 *  Created on: 30.7. 2016.
 *      Author: Vedran
 *
 *  Task scheduler library
 *  @version 2.4.1
 *  V1.1
 *  +Implementation of queue of tasks with various parameters. Tasks identified
 *      by unique integer number (defined by higher level library)
 *  V2.1 - 22.1.2017
 *  +Added time component to task entries - each task now has a time stamp at
 *      which it needs to be executed
 *  +Implemented SysTick in interrupt mode to count time from startup providing
 *      time reference for performing task at desired point in time from startup
 *  +Added callback registration for all kernel modules to register their services
 *  +Callback functionality from now on implemented so that module first registers
 *      its service by adding entry into the callback vector. Once the task
 *      scheduler requires that service it will transfer necessary memory into
 *      kernel space and call provided callback function for particular module
 *  V2.2
 *  +Switched to linked list as internal container for tasks - allows more
 *  flexibility in adding data (and can be sorted)
 *  V2.3 - 6.2.2016
 *  +TaskEntry instance now uses dynamically allocated array for storing arguments
 *  +Implemented support for periodic tasks. Once executed task is rescheduled
 *  based on its period. For non-periodic tasks period must be set to 0.
 *  V2.4 - 20.2.2017
 *  +Implemented repeat counter. Periodic tasks can now be automatically killed
 *  after a predefined number of repeats.
 *  V2.4.1 - 25.2.2017
 *  +TaskScheduler class now offers adding single arguments of basic data types
 *  (char, float...) through common template member-function AddArg(T arg)
 *  TODO:
 *  Implement UTC clock feature. If at some point program finds out what the
 *  actual time is it can save it and maintain real UTC time reference
 */
#include "roverKernel/hwconfig.h"

//  Compile following section only if hwconfig.h says to include this module
#if !defined(TASKSCHEDULER_H_) && defined(__HAL_USE_TASKSCH__)
#define TASKSCHEDULER_H_

#include <vector>

//  Enable debug information printed on serial port
//#define __DEBUG_SESSION__

/**
 * _taksEntry class - object wrapper for tasks handled by TaskScheduler class
 */
class TaskEntry
{
    // Functions & classes needing direct access to all members
	friend class TaskScheduler;
	friend void TSSyncCallback(void);
    friend void TS_GlobalCheck(void);
    friend class LinkedList;
	public:
		TaskEntry();
		TaskEntry(const TaskEntry& arg);
		TaskEntry(const volatile TaskEntry& arg);
		TaskEntry(uint8_t uid, uint8_t task, uint32_t time,
		          int32_t period = 0, int32_t repeats = 0);
		~TaskEntry();

		void        AddArg(void* arg, uint16_t argLen) volatile;

		         TaskEntry& operator= (const TaskEntry& arg);
        volatile TaskEntry& operator= (const volatile TaskEntry& arg);
        volatile TaskEntry& operator= (volatile TaskEntry& arg) volatile;

	protected:
		//  Unique identifier for library to request service from
		volatile uint8_t    _libuid;
		//  Service ID to execute
		volatile uint8_t    _task;
		//  Number of arguments provided when doing service call
		volatile uint16_t    _argN;
		//  Time at which to exec. service (in ms from start-up of task scheduler)
		volatile uint32_t   _timestamp;
		//  Arguments used when calling service - array that is dynamically
		//  allocated in AddArg function depending on the number of arguments
		volatile uint8_t    *_args;
		//  Period at which to execute this task (0 for non-periodic tasks)
		int32_t             _period;
		//  Number of times to repeat the task. When positive, defines how
		//  many repeats of that task remain, when negative, task
		//  will be repeated indefinitely. When == 0, task is killed.
		int32_t             _repeats;
};

/**
 * Node of data (of type TaskEntry) used in linked list
 * All member functions & constructors are private as this class shouldn't be
 * used outside the TaskScheduler object
 */
class _llnode
{
    friend class LinkedList;
    friend class TaskScheduler;

    private:
        _llnode();
        _llnode(volatile TaskEntry  &arg,
                volatile _llnode    *pre = 0,
                volatile _llnode    *nex = 0);

        volatile _llnode     *_prev,
                             *_next;
        volatile TaskEntry   data;
};

/**
 * Linked list of TaskEntry object
 * Linked list data container of sorted TaskEntry objects based on their
 * time stamp. Used only in TaskScheduler class to keep all pending task
 * requests ergo everything is private.
 */
class LinkedList
{
    friend class TaskScheduler;
    public:
        ~LinkedList();
    private:
        LinkedList();

        volatile _llnode*       AddSort(TaskEntry &arg) volatile;
        bool                    Empty() volatile;
        void                    Drop() volatile;
        TaskEntry               PopFront() volatile;
        volatile TaskEntry&     PeekFront() volatile;

    private:
        //  Volatile pointers as they might change inside ISRs
        volatile _llnode     * volatile head,
                             * volatile tail;
        const volatile TaskEntry   nullNode;
        volatile uint32_t    size;

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

		void 				 Reset() volatile;
		bool				 IsEmpty() volatile;
		void                 SyncTask(uint8_t libuid, uint8_t comm, int64_t time,
		                              bool periodic = false, int32_t rep = 0) volatile;
		void                 SyncTask(TaskEntry te) volatile;

		void                 AddArgs(void* arg, uint8_t argLen) volatile;

		/**
		 ****Template member function needs to be defined in the header file
		 * Add a single argument through the template function
		 * Allows to append argument of any type to the current task
		 * @note Once PopFront() function has been called it's not possible to append
		 * new arguments (because it's unknown if the _lastIndex node got deleted or not)
		 * @param arg data argument to append to the current task argument list
		 */
		template<typename T>
		void AddArg(T arg) volatile
		{
		    if (_lastIndex != 0)
		        _lastIndex->data.AddArg((void*)&arg, sizeof(arg));
		}

        TaskEntry            PopFront() volatile;
		volatile TaskEntry&  PeekFront() volatile;

	private:
		//  Queue of tasks to be executed
		volatile LinkedList	_taskLog;
		//  Pointer to last added item (need to be able to append arguments)
		//  ->Reset to zero after calling PopFront() function
		volatile _llnode    * volatile _lastIndex;
};

//  Global pointer to first instance of TaskScheduler object
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
 * @note IMPORTANT! 1st byte of args ALWAYS contains number of following data
 *  bytes, remember to use +1 offset in memory when doing memcpy on args array
 */
struct _kernelEntry
{
    void((*callBackFunc)(void));    // Pointer to callback function
    uint8_t serviceID;              // Requested service
    uint8_t *args;                  // Arguments for service execution
    uint16_t argN;                  // Length of *args array
    int32_t  retVal;                // (Optional) Return variable of service exec
};

extern void TS_RegCallback(struct _kernelEntry *arg, uint8_t uid);

//  Internal time since TaskScheduler startup (in ms)
extern volatile uint64_t msSinceStartup;

#endif /* TASKSCHEDULER_H_ */
