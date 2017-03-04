/**
 * taskScheduler.c
 *
 *  Created on: 30. 7. 2016.
 *      Author: Vedran
 */
#include "taskScheduler.h"

#if defined(__HAL_USE_TASKSCH__)   //  Compile only if module is enabled

#include "roverKernel/libs/myLib.h"
#include "roverKernel/HAL/hal.h"

#include <ctype.h>
#include "utils/uartstdio.h"


/**
 * Callback vector for all available kernel modules
 * Once a new kernel module is initialized it has a possibility to register its
 * service in task scheduler by providing designated callback function to be
 * called when requesting a service, and memory space for arguments to be
 * transfered to module when requesting a service
 */
static volatile struct _kernelEntry *__kernelVector[10];
/**
 * Register services for a kernel modules into a callback vector
 * @param arg structure with parameters for callback action
 * @param uid Unique identifier of kernel module
 */
void TS_RegCallback(struct _kernelEntry *arg, uint8_t uid)
{
    __kernelVector[uid] = arg;
}

//  Global pointer to FIRST instance of TaskScheduler object
volatile TaskScheduler* __taskSch;

///-----------------------------------------------------------------------------
///                      Class constructor & destructor                [PUBLIC]
///-----------------------------------------------------------------------------
TaskScheduler::TaskScheduler() : _lastIndex(0)
{
	if (__taskSch == 0) __taskSch = this;
}

TaskScheduler::~TaskScheduler()
{
    HAL_TS_StopSysTick();
	__taskSch = 0;
}

/**
 * Used to initialize hardware used by task scheduler (systick and interrupt)
 * In older version used to be called directly from constructor which would
 * require board clock to be configured at the time of initialization.
 */
void TaskScheduler::InitHW() volatile
{

    HAL_TS_InitSysTick(100, TSSyncCallback);
    HAL_TS_StartSysTick();
}

///-----------------------------------------------------------------------------
///                      Scheduler content manipulation                 [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Clear task schedule, remove all entries from it
 */
void TaskScheduler::Reset() volatile
{
    _taskLog.Drop();
}

/**
 * Return status of Task scheduler queue
 * @return	true: if there's nothing in queue
 * 		   false: if queue contains data
 */
bool TaskScheduler::IsEmpty() volatile
{
    return _taskLog.Empty();
}

/**
 * Add task to the task list in a sorted fashion (ascending sort). Tasks that
 * need to be executed sooner appear at the beginning of the list. If new task
 * has the same execution time as the task already in the list, it's placed
 * behind the existing task.
 * @param libuid UID of library to call
 * @param comm task ID within the library to execute
 * @param time time-stamp at which to execute the task. If >0 its absolute time
 * in ms since startup of task scheduler. If <=0 its relative time from NOW
 * @param rep repeat counter. Number of times to repeat the periodic task before
 * killing it. Set to a negative number for indefinite repeat. When scheduled,
 * task WILL BE repeated at least once.
 */
void TaskScheduler::SyncTask(uint8_t libuid, uint8_t comm,
                             int64_t time, bool periodic, int32_t rep) volatile
{
    int32_t period = (int32_t)time;
    /*
     * If time is a positive number it represent time in milliseconds from
     * start-up of the microcontroller. If time is a negative number or 0 it
     * represents a time in milliseconds from current time as provided by SysTick
     */
    if (time <= 0)
        time = (uint32_t)(-time) + msSinceStartup;
    else
        time = (uint32_t)time;

    //  Subtract 1 from number of repetition as 0 counts as actual repetition
    //  e.g. To repeat task 3 times (rep from arguments) task will be
    //  executed with index 2, 1 and 0
    if (rep > 0) rep--;

    //  Save pointer to newly added task so additional arguments can be appended
    //  to it through AddArgs function call
    TaskEntry teTemp(libuid, comm, time, (periodic?period:0), rep);
    _lastIndex = _taskLog.AddSort(teTemp);
}

/**
 * Add task to the task list in a sorted fashion (ascending sort). Tasks that
 * need to be executed sooner appear at the beginning of the list. If new task
 * has the same execution time as the task already in the list, it's placed
 * behind the existing task.
 * @param te TaskEntry object to add the the list
 */
void TaskScheduler::SyncTask(TaskEntry te) volatile
{
    //  Save pointer to newly added task so additional arguments can be appended
    //  to it through AddArgs function call
    _lastIndex = _taskLog.AddSort(te);
}

/**
 * Add arguments for the last pushed task. Any arguments added through here are
 * appended to the existing arguments provided for this task. So this function
 * can be repeatedly called to append multiple arguments.
 * @note Once PopFront() function has been called it's not possible to append
 * new arguments (because it's unknown if the _lastIndex node got deleted or not)
 * @param arg byte array of data to append (regardless of data type)
 * @param argLen size of byte array [arg]
 */
void TaskScheduler::AddArgs(void* arg, uint8_t argLen) volatile
{
    if (_lastIndex != 0)
        _lastIndex->data.AddArg(arg, argLen);
}

/**
 * Return first element from task queue
 * @note Once this function is called, _lastIndex pointer, that points to last
 * added task is set to 0 (because it's not possible to know whether that task
 * got deleted or no). This prevents calling AddArgs function until new task
 * is added
 * @return first element from task queue and delete it (by moving iterators).
 *          If the queue is empty it resets the queue.
 */
 TaskEntry TaskScheduler::PopFront() volatile
{
     TaskEntry retVal;
    retVal = _taskLog.PopFront();
    _lastIndex = 0;
    return retVal;
}

/**
 * Peek at the first element of task list but leave it in the list
 * @return reference to first task in task list
 */
volatile TaskEntry& TaskScheduler::PeekFront() volatile
{
    return _taskLog.head->data;
}

/*******************************************************************************
 *******************************************************************************
 *********             SysTick callback and time tracking              *********
 *******************************************************************************
 ******************************************************************************/

/// Internal time since TaskScheduler startup (in ms) - updated in SysTick ISR
volatile uint64_t msSinceStartup = 0;

/**
 * SysTick interrupt
 * Used to keep internal track of time either as number of milliseconds passed
 * from start-up of task scheduler or acquired UTC time
 */
void TSSyncCallback(void)
{
    msSinceStartup += HAL_TS_GetTimeStepMS();
}

/**
 * Task scheduler callback routine
 * This routine has to be called in order to execute tasks pushed in task queue
 * and is recently removed from TSSyncCallback because some task might rely on
 * interrupt routines that can't be executed while MCU is within TSSyncCallback
 * function which is a SysTick ISR. (no interrupts while in ISR)
 */
void TS_GlobalCheck(void)
{
    //  Check if there task scheduled to execute
    if (!__taskSch->IsEmpty())
        //  Check if the first task had to be executed already
        while((__taskSch->PeekFront()._timestamp <= msSinceStartup) &&
              (!__taskSch->IsEmpty()))
        {
            // Take out first entry to process it
            TaskEntry tE;
            tE = __taskSch->PopFront();

            // Check if callback exists
            if ((__kernelVector + tE._libuid) == 0)
                return;

            // Make task data available to kernel
            __kernelVector[tE._libuid]->serviceID = tE._task;
            __kernelVector[tE._libuid]->argN = tE._argN;
            __kernelVector[tE._libuid]->args = (uint8_t*)tE._args;

#if defined(__DEBUG_SESSION__)
            UARTprintf("Now is %d \n", msSinceStartup);

            UARTprintf("Processing %d:%d at %ul ms\n", tE._libuid, tE._task, tE._timestamp);
            UARTprintf("-(%d)> %s\n", tE._argN, tE._args);
#endif
            // Call kernel module to execute task
            __kernelVector[tE._libuid]->callBackFunc();

            //  If there's a period specified reschedule task
            if ((tE._period != 0) && (tE._repeats != 0))
            {
                //  If using repeat counter decrease it
                if (tE._repeats > 0)
                    tE._repeats--;
                //  Change time of execution based on period
                tE._timestamp = msSinceStartup + labs(tE._period);
                __taskSch->SyncTask(tE);
            }
        }
}

#endif  /* __HAL_USE_TASKSCH__ */
