/**
 * taskScheduler.c
 *
 *  Created on: 30. 7. 2016.
 *      Author: Vedran
 */
#include "taskScheduler.h"
#include "driverlib/interrupt.h"

#if defined(__HAL_USE_TASKSCH__)   //  Compile only if module is enabled

#include "libs/myLib.h"
#include "HAL/hal.h"

#include <ctype.h>

//  Enable debug information printed on serial port
//#define __DEBUG_SESSION__

//  Integration with event log, if it's present
#ifdef __HAL_USE_EVENTLOG__
    #include "init/eventLog.h"
    //  Simplify emitting events
    #define EMIT_EV(X, Y)  EventLog::EmitEvent(TASKSCHED_UID, X, Y)
#endif  /* __HAL_USE_EVENTLOG__ */

#ifdef __DEBUG_SESSION__
#include "serialPort/uartHW.h"
#endif

/**
 * Callback vector for all available kernel modules
 * Once a new kernel module is initialized it has a possibility to register its
 * service in task scheduler by providing designated callback function to be
 * called when requesting a service, and memory space for arguments to be
 * transfered to module when requesting a service
 */
static volatile struct _kernelEntry *__kernelVector[NUM_OF_MODULES] = {0};


/**
 * Register services for a kernel modules into a callback vector
 * @param arg structure with parameters for callback action
 * @param uid Unique identifier of kernel module
 */
void TS_RegCallback(struct _kernelEntry *arg, uint8_t uid)
{
    __kernelVector[uid] = arg;
}

//  Function prototype of an interrupt handler counting milliseconds since
//  startup(declared at the bottom)
void _TSSyncCallback();

/**
 * Callback routine to invoke service offered by this module from task scheduler
 * @note It is assumed that once this function is called task scheduler has
 * already copied required variables into the memory space provided for it.
 */
void _TS_KernelCallback(void)
{
    volatile TaskScheduler  &__ts = TaskScheduler::GetI();

    //  Check for null-pointer
    if (__ts._tsKer.args == 0)
        return;

    /*
     *  Data in args[] contains bytes that constitute arguments for function
     *  calls. The exact representation(i.e. whether bytes represent ints, floats)
     *  of data is known only to individual blocks of switch() function. There
     *  is no predefined data separator between arguments inside args[].
     */
    switch (__ts._tsKer.serviceID)
    {
    /*
     *  Enable/disable time ticking on internal timer
     *  args[] = enable(bool)
     *  retVal on of myLib.h STATUS_* macros
     */
    case TASKSCHED_T_ENABLE:
        {
            bool enable = (__ts._tsKer.args[0] == 1);

            if (enable)
                HAL_TS_StartSysTick();
            else
                HAL_TS_StopSysTick();

            __ts._tsKer.retVal = STATUS_OK;
        }
        break;
    /*
     *  Delete task by its PID
     *  args[] = taskPID(uint16_t)
     *  retVal on of myLib.h STATUS_* macros
     */
    case TASKSCHED_T_KILL:
        {
            uint16_t PIDarg;

            memcpy(&PIDarg, __ts._tsKer.args, sizeof(uint16_t));

            __ts.RemoveTask(PIDarg);

            __ts._tsKer.retVal = STATUS_OK;
        }
        break;
    default:
        break;
    }

    //  Check return-value and emit event based on it
#ifdef __HAL_USE_EVENTLOG__
    if (__ts._tsKer.retVal == STATUS_OK)
        EMIT_EV(__ts._tsKer.serviceID, EVENT_OK);
    else
        EMIT_EV(__ts._tsKer.serviceID, EVENT_ERROR);
#endif  /* __HAL_USE_EVENTLOG__ */
}

///-----------------------------------------------------------------------------
///         Functions for returning static instance                     [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Return reference to a singleton
 * @return reference to an internal static instance
 */
 volatile TaskScheduler& TaskScheduler::GetI()
{
    static volatile TaskScheduler singletonInstance;
    return singletonInstance;
}

/**
 * Return pointer to a singleton
 * @return pointer to a internal static instance
 */
volatile TaskScheduler* TaskScheduler::GetP()
{
    return &(TaskScheduler::GetI());
}

///-----------------------------------------------------------------------------
///                      Class member function definitions              [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Validate that a kernel module with libUID is registered within kernel
 * @param libUID UID of a kernel module to validate
 * @return true if registered, false otherwise
 */
bool TaskScheduler::ValidKernModule(uint8_t libUID)
{
    return (__kernelVector[libUID] != 0);
}

/**
 * Used to initialize hardware used by task scheduler (systick and interrupt)
 * In older version used to be called directly from constructor which would
 * require board clock to be configured at the time of initialization.
 * @param timeStepMS internal time step (in ms) by which internal time is
 * increased every systick (also a period of systick)
 */
void TaskScheduler::InitHW(uint32_t timeStepMS) volatile
{
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_STARTUP);
#endif  /* __HAL_USE_EVENTLOG__ */

    //  Initialize & start systick => keeps internal time reference
    HAL_TS_InitSysTick(timeStepMS, _TSSyncCallback);
    HAL_TS_StartSysTick();

    //  Register module services with task scheduler
    _tsKer.callBackFunc = _TS_KernelCallback;
    TS_RegCallback((struct _kernelEntry*)&_tsKer, TASKSCHED_UID);

#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_INITIALIZED);
#endif  /* __HAL_USE_EVENTLOG__ */
}

///-----------------------------------------------------------------------------
///                      Scheduler content manipulation                 [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Clear task schedule, remove all entries from it
 */
void TaskScheduler::Reset() volatile
{
    //  Sensitive task, disable all interrupts
    IntMasterDisable();

    //  If Drop() return true, there was an error deleting tasks
    if (_taskLog.Drop())
        EMIT_EV(-1, EVENT_ERROR);

    //  Sensitive task done, enable interrupts again
    IntMasterEnable();
}

/**
 * Return number of tasks currently pending execution
 * @return Current number of tasks in task list
 */
uint32_t TaskScheduler::NumOfTasks() volatile
{
    return _taskLog.size;
}

/**
 * This is implemented solely for the purpose of printing out task in task
 * scheduler. First call should be made with argument true and all consecutive
 * calls with arg false in order to get all tasks on the list out.
 * @param fromStart True to start returning from head of linked list, false to
 * return next element
 * @return TaskEntry element from the list; index corresponds to a number of
 * calls to this function since last fromStart was 'true'. If index is out of
 * boundaries, 0 (check for null pointer on exit)
 */
const TaskEntry* TaskScheduler::FetchNextTask(bool fromStart) volatile
{
    static _llnode *task = 0;


    //
    if (fromStart)
        task = (_llnode*)(_taskLog.head);
    else if (task->_next != 0)
        task = (_llnode*)(task->_next);
//    else
//        return 0;

    return (TaskEntry*)(&(task->data));
}

/**
 * Add task to the task list in a sorted fashion (ascending sort). Tasks that
 * need to be executed sooner appear at the beginning of the list. If new task
 * has the same execution time as the task already in the list, it's placed
 * behind the existing task.
 * @param libUID UID of library to call
 * @param taskID task ID within the library to execute
 * @param time time-stamp at which to execute the task. If >0 its absolute time
 * in ms since startup of task scheduler. If <=0 its relative time from NOW
 * @param periodic If true, schedules periodic task with provided number of
 * repeats. Period is absolute value of 'time' parameter
 * @param rep repeat counter. Number of times to repeat the periodic task before
 * killing it. Set to a negative number for indefinite repeat. When scheduled,
 * task WILL BE repeated at least once.
 */
void TaskScheduler::SyncTask(uint8_t libUID, uint8_t taskID,
                             int64_t time, bool periodic, int32_t rep) volatile
{
    //  Sensitive task, disable all interrupts
    IntMasterDisable();

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
    TaskEntry teTemp(libUID, taskID, time, (periodic?period:0), rep);
#if defined(__DEBUG_SESSION2__)
        volatile uint32_t siz = _taskLog.size;
#endif
    _lastIndex = _taskLog.AddSort(teTemp);
#if defined(__DEBUG_SESSION2__)
        if ((_taskLog.size-siz) != 1)
        {
            DEBUG_WRITE("\nNow is %d, STA\n", msSinceStartup);
            DEBUG_WRITE("  Adding %d(%d) \n", libUID, taskID);
            DEBUG_WRITE("  Size before %d \n", siz);
            DEBUG_WRITE("  Size after %d \n", _taskLog.size);

            int i = 0;
            while(i < _taskLog.size)
            {
                const TaskEntry *task = FetchNextTask(i==0);
                if (task == 0)
                    break;
                DEBUG_WRITE("    %d.[%u]: %d(%d)\n", i,(uint32_t)task->_timestamp, task->_libuid, task->_task);

                i++;
            }
            EMIT_EV(-1, EVENT_ERROR);
        }
#endif
        //  Sensitive task done, enable interrupts again
        IntMasterEnable();
}

/**
 * Add periodic task to the task list in a sorted fashion (ascending sort). Tasks
 * that need to be executed sooner appear at the beginning of the list. If new
 * task has the same execution time as the task already in the list, it's placed
 * behind the existing task.
 * @param libUID UID of library to call
 * @param taskID task ID within the library to execute
 * @param time time-stamp at which to execute the task. If >0 its absolute time
 * in ms since startup of task scheduler. If <=0 its relative time from NOW
 * @param period Period at which to repeat task
 * @param rep repeat counter. Number of times to repeat the periodic task before
 * killing it. Set to a negative number for indefinite repeat. When scheduled,
 * task WILL BE repeated at least once.
 */
void TaskScheduler::SyncTaskPer(uint8_t libUID, uint8_t taskID, int64_t time,
                      int32_t period, int32_t rep) volatile
{
    //  Sensitive task, disable all interrupts
    IntMasterDisable();
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
    TaskEntry teTemp(libUID, taskID, time, period, rep);
#if defined(__DEBUG_SESSION2__)
        volatile uint32_t siz = _taskLog.size;
#endif
    _lastIndex = _taskLog.AddSort(teTemp);
#if defined(__DEBUG_SESSION2__)
        if ((_taskLog.size-siz) != 1)
        {
            DEBUG_WRITE("\nNow is %d, STA\n", msSinceStartup);
            DEBUG_WRITE("  Adding %d(%d) \n", libUID, taskID);
            DEBUG_WRITE("  Size before %d \n", siz);
            DEBUG_WRITE("  Size after %d \n", _taskLog.size);

            int i = 0;
            while(i < _taskLog.size)
            {
                const TaskEntry *task = FetchNextTask(i==0);
                if (task == 0)
                    break;
                DEBUG_WRITE("    %d.[%u]: %d(%d)\n", i,(uint32_t)task->_timestamp, task->_libuid, task->_task);

                i++;
            }
            EMIT_EV(-1, EVENT_ERROR);
        }
#endif
        //  Sensitive task done, enable interrupts again
        IntMasterEnable();
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
    //  Sensitive task, disable all interrupts
    IntMasterDisable();

#if defined(__DEBUG_SESSION2__)
        volatile uint32_t siz = _taskLog.size;
#endif
        //  Save pointer to newly added task so additional arguments can be
        //  appended to it through AddArgs function call
        _lastIndex = _taskLog.AddSort(te);
#if defined(__DEBUG_SESSION2__)
        if ((_taskLog.size-siz) != 1)
        {
            DEBUG_WRITE("\nNow is %d, ST\n", msSinceStartup);
            DEBUG_WRITE("  Adding %d(%d) \n", te._libuid, te._task);
            DEBUG_WRITE("  Size before %d \n", siz);
            DEBUG_WRITE("  Size after %d \n", _taskLog.size);

            int i = 0;
            while(i < _taskLog.size)
            {
                const TaskEntry *task = FetchNextTask(i==0);
                if (task == 0)
                    break;
                DEBUG_WRITE("    %d.[%u]: %d(%d)\n", i,(uint32_t)task->_timestamp, task->_libuid, task->_task);

                i++;
            }
            EMIT_EV(-1, EVENT_ERROR);
        }
#endif
        //  Sensitive task done, enable interrupts again
        IntMasterEnable();
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
void TaskScheduler::AddArgs(void* arg, uint16_t argLen) volatile
{
    //  Sensitive task, disable all interrupts
    IntMasterDisable();

    if (_lastIndex != 0)
        _lastIndex->data.AddArg(arg, argLen);

    //  Sensitive task done, enable interrupts again
    IntMasterEnable();
}

/**
 * Find and delete the task in task list matching these arguments
 * @param libUID
 * @param taskID
 * @param arg
 * @param argLen
 */
void TaskScheduler::RemoveTask(uint8_t libUID, uint8_t taskID,
                               void* arg, uint16_t argLen) volatile
{
    //  Sensitive task, disable all interrupts
    IntMasterDisable();

    TaskEntry delT(libUID, taskID, 0);
    delT.AddArg(arg, argLen);
    _taskLog.RemoveEntry(delT);

    //  Sensitive task done, enable interrupts again
    IntMasterEnable();
}

/**
 *
 * @param libUID
 * @param taskID
 * @param arg
 * @param argLen
 * @return
 */
/**
 * Find and delete the task in task list matching a given PID
 * @param PIDarg PID (Unque process ID) of task to kill
 * @return true if removed; false otherwise()
 */
bool TaskScheduler::RemoveTask(uint16_t PIDarg) volatile
{
    bool retVal;
    //  Sensitive task, disable all interrupts
    IntMasterDisable();

    retVal = _taskLog.RemoveEntry(PIDarg);

    //  Sensitive task done, enable interrupts again
    IntMasterEnable();
    return retVal;
}

///-----------------------------------------------------------------------------
///                      Class constructor & destructor              [PROTECTED]
///-----------------------------------------------------------------------------
TaskScheduler::TaskScheduler() : _lastIndex(0)
{
#ifdef __HAL_USE_EVENTLOG__
    EMIT_EV(-1, EVENT_UNINITIALIZED);
#endif  /* __HAL_USE_EVENTLOG__ */
}

TaskScheduler::~TaskScheduler()
{
    HAL_TS_StopSysTick();
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
void _TSSyncCallback(void)
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
    //  Grab reference to singleton
    volatile TaskScheduler &__taskSch = TaskScheduler::GetI();

    //  Check if there is task scheduled to execute
    if (!__taskSch.IsEmpty())
        //  Check if the first task had to be executed already
        while((__taskSch.PeekFront()._timestamp <= msSinceStartup) &&
              (!__taskSch.IsEmpty()))
        {
            // Take out first entry to process it
            TaskEntry tE(__taskSch.PopFront());
            uint64_t tStart = (uint64_t)msSinceStartup;

            //  If we're going to repeat this task then it makes sense to
            //  measure its performance, run task-start hook  and calculate new
            //  starting time for this task
            if ((tE._period != 0) && (tE._repeats != 0))
            {
#ifdef _TS_PERF_ANALYSIS_
                tE._perf.TaskStartHook((uint64_t)msSinceStartup, tE._timestamp, HAL_TS_GetTimeStepMS());
#endif
                //  Change time of execution based on period (for next execution)
                tE._timestamp = msSinceStartup + labs(tE._period);
            }

            // Check if module is registered in task scheduler
            if ((__kernelVector[tE._libuid]) == 0)
                return;

#if defined(__DEBUG_SESSION__)
            DEBUG_WRITE("Now is %d \n", msSinceStartup);

            DEBUG_WRITE("Processing %d:%d at %ul ms\n", tE._libuid, tE._task, tE._timestamp);
            DEBUG_WRITE("-(%d)> %s\n", tE._argN, tE._args);
#endif

            // Make task data available to kernel
            __kernelVector[tE._libuid]->serviceID = tE._task;
            __kernelVector[tE._libuid]->argN = tE._argN;
            __kernelVector[tE._libuid]->args = (uint8_t*)tE._args;

            // Call kernel module to execute task
            __kernelVector[tE._libuid]->callBackFunc();

            //  If there's a period specified, reschedule task
            //  Run post-execution hook for calculating performance
            if ((tE._period != 0) && (tE._repeats != 0))
            {
#ifdef _TS_PERF_ANALYSIS_
                tE._perf.TaskEndHook((uint64_t)msSinceStartup);
#endif
                //  If using repeat counter decrease it
                if (tE._repeats > 0)
                    tE._repeats--;
                //  Reschedule the task
                __taskSch.SyncTask(tE);
            }
        }
}


#endif  /* __HAL_USE_TASKSCH__ */
