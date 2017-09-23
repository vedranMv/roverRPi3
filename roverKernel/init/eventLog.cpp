/**
 * eventLog.cpp
 *
 *  Created on: Jul 2, 2017
 *      Author: Vedran
 */
#include "eventLog.h"

//  Enable debug information printed on serial port
//#define __DEBUG_SESSION__

#if defined(__HAL_USE_EVENTLOG__)   //  Compile only if module is enabled

//  Simplify emitting events
#define EMIT_EV(X, Y)  EventLog::EmitEvent(EVLOG_UID, X, Y)

/**
 * Callback routine to invoke service offered by this module from task scheduler
 * @note It is assumed that once this function is called task scheduler has
 * already copied required variables into the memory space provided for it.
 */
void _EVLOG_KernelCallback()
{
    //  Grab a pointer to singleton
    EventLog &__evlog = EventLog::GetI();

    //  Check for null-pointer
    if (__evlog._evlogKer.argN == 0)
        return;
    /*
     *  Data in args[] contains bytes that constitute arguments for function
     *  calls. The exact representation(i.e. whether bytes represent ints, floats)
     *  of data is known only to individual blocks of switch() function. There
     *  is no predefined data separator between arguments inside args[].
     */
    switch (__evlog._evlogKer.serviceID)
    {
    /*
     * Drop all data in event log before given timestamp (in milliseconds)
     * First 4 bytes contain time in ms as given by the task scheduler
     * args[] = timestamp(uint32_t)
     * retVal one of myLib.h STATUS_* error codes
     */
    case EVLOG_DROP:
        {
            uint32_t timestamp;
            memcpy(&timestamp, __evlog._evlogKer.args, sizeof(uint32_t));

            __evlog._evlogKer.retVal = __evlog.DropBefore(timestamp);
        }
        break;
    /*
     * Perform full reboot of event log, completely deleting all data in it
     * args[] = accessCode(0x17)
     * retVal one of myLib.h STATUS_* error codes
     */
    case EVLOG_REBOOT:
        {
            //  Reboot only if 0x17 was sent as access code
            if (__evlog._evlogKer.args[0] != 0x17)
                return;
            __evlog._evlogKer.retVal = __evlog.Reset();

            EMIT_EV(__evlog._evlogKer.serviceID, EVENT_INITIALIZED);
        }
        break;
        /*
         * Perform soft reboot (only event logger status) for specified module
         * args[] = accessCode(0xCF)|libUID
         * retVal one of myLib.h STATUS_* error codes
         */
    case EVLOG_SOFT_REBOOT:
        {
            //  Soft reboot access code is 0xCF, skip if it's not valid
            if (__evlog._evlogKer.args[0] != 0xCF)
                return;
            //  Perform soft reboot only if the module exists, otherwise we risk
            //  fault
            if (TaskScheduler::ValidKernModule(__evlog._evlogKer.args[1]));
                EventLog::SoftReboot(__evlog._evlogKer.args[1]);
        }
        break;
    default:
        break;
    }

    //  Emit event corresponding to result of task execution
    if (__evlog._evlogKer.retVal != STATUS_OK)
        EMIT_EV(__evlog._evlogKer.serviceID, EVENT_ERROR);
    else
        EMIT_EV(__evlog._evlogKer.serviceID, EVENT_OK);
}

///-----------------------------------------------------------------------------
///         Functions for returning static instance                     [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Return reference to a singleton
 * @return reference to an internal static instance
 */
EventLog& EventLog::GetI()
{
    static EventLog singletonInstance;
    return singletonInstance;
}

/**
 * Return pointer to a singleton
 * @return pointer to a internal static instance
 */
EventLog* EventLog::GetP()
{
    return &(EventLog::GetI());
}

/**
 * Initialize software used by the event logger
 * Registers this module with task scheduler to allow execution of remote tasks
 */
void EventLog::InitSW()
{
    EMIT_EV(-1, EVENT_STARTUP);
#if defined(__USE_TASK_SCHEDULER__)
    //  Register module services with task scheduler
    _evlogKer.callBackFunc = _EVLOG_KernelCallback;
    TS_RegCallback(&_evlogKer, EVLOG_UID);
#endif
    EMIT_EV(-1, EVENT_INITIALIZED);
}

///-----------------------------------------------------------------------------
///         Functions for manipulating event log                        [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Record events control
 * Allow for enabling or disabling of event-logging. By default all system
 * events are logged.
 * @param enable Whether to record emitted system events or not
 */
void EventLog::RecordEvents(bool enable)
{
    _enSig = enable;
}

/**
 * Interface for logging events
 * Called by all system modules when they want to log an event. Function
 * constructs event entry from provided arguments, appends current timestamp to
 * it and saves it in linked list stored in EventLog class.
 * @param libUID ID of module which emitted event
 * @param taskID ID of task which was being executed when event occurred
 * @param event One of EVENT_* enums from header file, describing event
 */
void EventLog::EmitEvent(uint8_t libUID, int8_t taskID, Events event)
{
    //  Get reference of singleton
    EventLog &el = EventLog::GetI();

    //  If event logger is not enabled stop here
    if (!el._enSig)
        return;

    //  Check if log is over its allowed size, remove all log entries if log is full
    if (el._entryvCount > MAX_LOG_ENTRIES)
        el.DropBefore(0xFFFFFFFF);

    //  Allocate new memory for linked least
    static volatile struct _eventEntry *lastEntry = 0;
    volatile struct _eventEntry *eventInst = new struct _eventEntry,
                                *prioInvEvent = 0;

    //  Populate event instance with event data
    eventInst->libUID = libUID;
    eventInst->taskID = taskID;
    eventInst->timestamp = msSinceStartup;
    eventInst->event = event;
    eventInst->next = 0;

    //  Prevent repeated logging of same events within a module
    //  Check if the same event for this module has already been logged on the
    //  last function call, if so add this event only if enough time has
    //  passed between those two events
    if ((el._lastEvent[libUID].event == event) &&
        ((eventInst->timestamp-el._lastEvent[libUID].timestamp) < REP_TIME_DIFF_MS))
    {
        //  If there wasn't enough time remove the object from heap and stop here
        delete eventInst;
        return;
    }

    //  If current event is the new highest priority one, save it       OR
    //  If it's a startup event, save it as new high prio. one thereby resetting
    //  the highest priority entry for this module
    if ((event >= el._highestPrioEv[libUID].event) || (event == EVENT_STARTUP))
    {
        el._highestPrioEv[libUID] = *((struct _eventEntry *)eventInst);
        el._prioInvOcc[libUID] = false;
    }
    else
    {
        //  If there is higher priority event than this already logged, module
        //  experienced priority inversion. Add original event in the list, but
        //  also add priority inversion event after it.
        prioInvEvent = new struct _eventEntry;
        prioInvEvent->libUID = libUID;
        prioInvEvent->taskID = taskID;
        prioInvEvent->timestamp = msSinceStartup;
        prioInvEvent->event = EVENT_PRIOINV;
        prioInvEvent->next = 0;
        el._prioInvOcc[libUID] = true;
    }

    //  Adding event to a linked list of EventLog
    //  If it's not first entry update 'next' pointer of previous entry
    if (el._entryVectorHead == 0)
        el._entryVectorHead = eventInst;
    else
        lastEntry->next = eventInst;

    //  Save current entry to be used as new 'previous' entry in next run
    lastEntry = eventInst;
    el._lastEvent[libUID] = *((struct _eventEntry *)eventInst);
    el._entryvCount++;

    //  Check if there's an event for priority inversion that needs to be added
    if (prioInvEvent != 0)
    {
        lastEntry->next = prioInvEvent;
        lastEntry = prioInvEvent;
        el._entryvCount++;
    }
}

/**
 * Drop all entries in event log before given timestamp
 * @param timestamp Time in ms (as reported by task scheduler module) before
 * which all events are to be erased from event log
 * @return One of myLib.h STATUS_* error codes
 */
uint32_t EventLog::DropBefore(uint32_t timestamp)
{
    uint32_t retVal = STATUS_OK;
    //  Delete dynamically allocated list
   volatile struct _eventEntry *node = _entryVectorHead;

   //   Loop through the list and delete everything with time before timestamp
   while (node != 0)
   {
       //   Check if end condition has been reached
       if (node->timestamp > timestamp)
           break;

       //   Save pointer to next element
       volatile struct _eventEntry *tmp = node->next;
       //   Delete current element
       delete node;
       //    Set next as new current element
       node = tmp;
       //   Decrease number of entries in list
       _entryvCount--;
   }

   _entryVectorHead = node;

   return retVal;
}

/**
 * Perform full reset of event log, clear all logs, reset all saved states that
 * are usually not deleter with DropBefore() function call
 * @return One of myLib.h STATUS_* error codes
 */
uint32_t EventLog::Reset()
{
    uint32_t retVal = STATUS_OK;

    //  Delete dynamically allocated list
   volatile struct _eventEntry *node = _entryVectorHead;

   //   Loop through the list until node is null pointer
   while (node != 0)
   {
       //   Save pointer to next element
       volatile struct _eventEntry *tmp = node->next;
       //   Delete current element
       delete node;
       //    Set next as new current element
       node = tmp;
       //   Decrease number of entries in list
       _entryvCount--;
   }

   //   If we've reached end of the list but number of items in the list is !=0
   //   we have memory leakage, raise event
   if(_entryvCount != 0)
   {
       EMIT_EV(-1, EVENT_ERROR);
       retVal = STATUS_PROG_ERR;
   }

   //   Construct empty task entry for resetting event arrays
   struct _eventEntry empty;
   empty.event = EVENT_UNINITIALIZED;
   empty.timestamp = 0;
   empty.libUID = -1;
   empty.taskID = -1;
   //   Reset also arrays keeping events when main log is deleted
   for (uint8_t i = 0; i < NUM_OF_MODULES; i++)
   {
       _lastEvent[i] = empty;
       _highestPrioEv[i] = empty;
       _prioInvOcc[i] = false;
   }

   return retVal;
}

/**
 * Perform soft-reboot sequence on a given kernel module
 * @param libUID
 */
void EventLog::SoftReboot(uint8_t libUID)
{
    EmitEvent(libUID, -1, EVENT_STARTUP);
    EmitEvent(libUID, -1, EVENT_INITIALIZED);
}

///-----------------------------------------------------------------------------
///         Functions for accessing event log                           [PUBLIC]
///-----------------------------------------------------------------------------

/**
 * Return number of events currently in the log
 * @return
 */
uint16_t EventLog::EventCount()
{
    return _entryvCount;
}

/**
 * Get first node of the linked list containing event log
 * @return
 */
volatile struct _eventEntry* EventLog::GetHead()
{
    return _entryVectorHead;
}

struct _eventEntry EventLog::GetLastEvAt(uint8_t index)
{
        return _lastEvent[index];
}
struct _eventEntry EventLog::GetHigPrioEvAt(uint8_t index)
{
        return _highestPrioEv[index];
}
bool EventLog::GetPrioInvAt(uint8_t index)
{
        return _prioInvOcc[index];
}
///-----------------------------------------------------------------------------
///                      Class constructor & destructor              [PROTECTED]
///-----------------------------------------------------------------------------

EventLog::EventLog() : _entryVectorHead(0), _entryvCount(0), _enSig(true)
{
    for (int i = 0; i < NUM_OF_MODULES; i++)
    {
        _lastEvent[i].libUID = -1;
        _highestPrioEv[i].libUID = -1;
        _prioInvOcc[i] = false;
    }
}

EventLog::~EventLog()
{
    Reset();
}

#endif  /* __HAL_USE_EVENTLOG__ */
