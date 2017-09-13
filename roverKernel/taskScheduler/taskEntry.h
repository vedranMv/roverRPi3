/**
 * taskEntry.c
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran Mikov
 */
#ifndef ROVERKERNEL_TASKSCHEDULER_TASKENTRY_C_
#define ROVERKERNEL_TASKSCHEDULER_TASKENTRY_C_

#include "libs/myLib.h"

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
    friend void _PLAT_KernelCallback(void);
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
        //  Unique process ID
        volatile uint16_t _PID;
};

#endif /* ROVERKERNEL_TASKSCHEDULER_TASKENTRY_C_ */
