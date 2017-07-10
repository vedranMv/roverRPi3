/**
 * eventLog.h
 *
 *  Created on: Jul 2, 2017
 *      Author: Vedran Mikov
 *
 *  Event logger singleton is thought of as a mechanism to report status from
 *  tasks executed by a task scheduler. Because the idea with tasks scheduler is
 *  that tasks get scheduled in advanced and the issuer of the task doesn't
 *  wait until the task is completed, there is generally no way of telling how
 *  did the task perform. Event logger then provides a way of reporting the
 *  execution outcome by collecting system-wide events into a single linked list,
 *  noting which module emitted the event, at what time and during execution of
 *  which task. Event log is deleted after reaching certain size but most
 *  important information(highest-priority event since startup, last emitted
 *  event and appearance of priority inversion) about events from each module
 *  get remembered even after dropping the log.
 *
 *  @version 1.1.1
 *  V1.0.0 - 2.7.2017
 *  +Support 6 events that can be emitted by different libraries
 *  +Integrated with task scheduler for remote emptying of log
 *  V1.1.0 - 6.7.2017
 *  +Keep track of highest-importance event from startup event of each module
 *  +Prevent spamming of events by logging same, consecutive events only after
 *  a fixed period of time has passed. (only for same, consecutive events
 *  within the same library; taskID doesn't matter)
 *  +Event log can exclusively emit priority-inversion event when module
 *  emits low-priority event after a higher-priority event has already been
 *  logged since last start-up event. Reemitting startup event clears any
 *  previous event of higher priority
 *  +Fixed log length. Log is dropped when limit is reached (last event from
 *  each module is saved, together with priority inversion occurrence and highest
 *  priority event since startup in each module)
 *  V1.1.1 - 9.7.2017
 *  +Added more tasks accessible from task scheduler
 *  +Exposed arrays with last logged event, highest prio. event and prio. inversion
 *  through EventLog member functions
 *  +Added 'Reboot' task to completely clear event logger through task scheduler
 *
 */
#include "hwconfig.h"
#if !defined(ROVERKERNEL_INIT_EVENTLOG_H_) \
    && defined(__HAL_USE_EVENTLOG__)
#define ROVERKERNEL_INIT_EVENTLOG_H_


//  Enable integration of this library with task scheduler but only if task
//  scheduler is being compiled into this project
#if defined(__HAL_USE_TASKSCH__)
#define __USE_TASK_SCHEDULER__
#endif  /* __HAL_USE_TASKSCH__ */

//  Check if this library is set to use task scheduler
#if defined(__USE_TASK_SCHEDULER__)
    #include "taskScheduler/taskScheduler.h"
    //  Unique identifier of this module as registered in task scheduler
    #define EVLOG_UID            6
    //  Definitions of ServiceID for service offered by this module
    #define EVLOG_DROP           0
    #define EVLOG_REBOOT         1
#endif

//  Defines minimum time difference between two same events of a single module
//  to be logged - prevents unnecessary logging of same events happening fast
#define REP_TIME_DIFF_MS    300000      //  5 minutes
//  After reaching max number of entries log is dropped to save memory
//  max 200*(64+8+8+8)=17600bytes on the heap
#define MAX_LOG_ENTRIES     100

/**
 * Events that modules can transmit
 * Priority inversion event can only be set by EventLogger and it occurs when
 * a module has emitted a higher priority event (such as EVENT_ERROR) followed by
 * a lower priority event (such as EVENT_OK) without going through process of
 * reinitialization.
 */
enum Events {EVENT_UNINITIALIZED,   //Module is not yet initialized
             EVENT_STARTUP,         //Module is in startup sequence
             EVENT_INITIALIZED,     //Module is fully initialized
             EVENT_OK,              //Module performed request
             EVENT_HANG,            //Module is hanging in communication with HW
             EVENT_ERROR,           //Module experienced error
             EVENT_PRIOINV };       //Priority inversion event

/**
 * Single event entry in event log
 * Event log implemented as a linked list, 'next' points to next element in list
 */
struct _eventEntry
{
        uint64_t timestamp; //  Time in ms since startup when event was emitted
        int8_t libUID;      //  Module that emitted event
        int8_t taskID;      //  Task within module that emitted event
        Events  event;      //  Emitted event

        volatile struct _eventEntry *next;
};

/**
 * EventLog class definition
 * Object provides system-wide monitoring of events occurring in different
 * modules.
 */
class EventLog
{
    friend void _EVLOG_KernelCallback(void);

    public:
        //  Functions for returning static instance
        static EventLog& GetI();
        static EventLog* GetP();

        //  Initialization of SW for event log
        void            InitSW();
        //  Functions for manipulating event log
        void            RecordEvents(bool enable);
        static void     EmitEvent(uint8_t libUID, int8_t taskID, Events event);
        uint32_t        DropBefore(uint32_t timestamp);
        uint32_t        Reset();
        //  Functions for accessing event log
        uint16_t                        EventCount();
        volatile struct _eventEntry*    GetHead();
        struct _eventEntry              GetLastEvAt(uint8_t index);
        struct _eventEntry              GetHigPrioEvAt(uint8_t index);
        bool                            GetPrioInvAt(uint8_t index);

    protected:
        EventLog();
        ~EventLog();
        EventLog(EventLog &arg) {}              //  No definition - forbid this
        void operator=(EventLog const &arg) {}  //  No definition - forbid this

        //  Head of linked list with events
        volatile struct _eventEntry *_entryVectorHead;
        //  Number of events in linked list
        uint16_t                     _entryvCount;
        //  Enable signal for event logger, events are logged only when _enSig=true
        bool                         _enSig;
        //  Last recorded event for each module
        struct _eventEntry  _lastEvent[NUM_OF_MODULES];
        //  Highest priority event for each module since last EVENT_STARTUP
        struct _eventEntry  _highestPrioEv[NUM_OF_MODULES];
        //  Goes true whenever a priority inversion has occurred in a module
        bool                _prioInvOcc[NUM_OF_MODULES];

    //  Interface with task scheduler - provides memory space and function
    //  to call in order for task scheduler to request service from this module
#if defined(__USE_TASK_SCHEDULER__)
        _kernelEntry _evlogKer;
#endif
};


#endif /* ROVERKERNEL_INIT_EVENTLOG_H_ */
