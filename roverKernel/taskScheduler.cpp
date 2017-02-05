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


/*
 * Global pointer to FIRST instance of TaskScheduler object
 * (doesn't have to be volatile as this function shouldn't be called from ISR)
 */
volatile TaskScheduler* __taskSch;

/*******************************************************************************
 *******************************************************************************
 *********            TaskEntry class member functions                *********
 *******************************************************************************
 ******************************************************************************/

///-----------------------------------------------------------------------------
///                      Class constructors                             [PUBLIC]
///-----------------------------------------------------------------------------
TaskEntry::TaskEntry() : _libuid(0), _task(0), _argN(0), _timestamp(0)
{
    memset((void*)_args, 0, TS_TASK_MEMORY);
}

TaskEntry::TaskEntry(uint8_t uid, uint8_t task, uint32_t time)
            :_libuid(uid), _task(task), _timestamp(time),  _argN(0)
{
    memset((void*)_args, 0, TS_TASK_MEMORY);
}

TaskEntry::TaskEntry(const TaskEntry& arg) :  _argN(0)
{
    *this = (const TaskEntry&)arg;
}

TaskEntry::TaskEntry(const volatile TaskEntry& arg) :  _argN(0)
{
    *this = (const volatile TaskEntry&)arg;
}


/**
 * Add argument(s) stored in a byte array [arg] of length [argLen]. Byte array
 * may contain data of any type, as long as receiver of that data knows how to
 * interpret bytes stored in the field
 * @param arg byte array of data to pass to the function
 * @param argLen length of byte array [arg] (in bytes)
 */
void TaskEntry::AddArg(void* arg, uint8_t argLen) volatile
{
    memcpy((void*)(_args+_argN), arg, argLen);
    _argN += argLen;
}

/**
 * Class assignment operators for various combinations of data types
 * @param arg right side of equal-sign
 * @return
 */
TaskEntry& TaskEntry::operator= (const TaskEntry& arg)
{
    _libuid = arg._libuid;
    _task = arg._task;
    _argN = arg._argN;
    _timestamp = arg._timestamp;

    for (uint8_t i = 0; i < sizeof(_args); i++)
        _args[i] = arg._args[i];
    return *this;
}

volatile TaskEntry& TaskEntry::operator= (const volatile TaskEntry& arg)
{
    _libuid = arg._libuid;
    _task = arg._task;
    _argN = arg._argN;
    _timestamp = arg._timestamp;

    for (uint8_t i = 0; i < sizeof(_args); i++)
        _args[i] = arg._args[i];
    return *this;
}

volatile TaskEntry& TaskEntry::operator= (volatile TaskEntry& arg) volatile
{
    _libuid = arg._libuid;
    _task = arg._task;
    _argN = arg._argN;
    _timestamp = arg._timestamp;

    for (uint8_t i = 0; i < sizeof(_args); i++)
        _args[i] = arg._args[i];
    return (volatile TaskEntry&) *this;
}

/*******************************************************************************
 *******************************************************************************
 *********     Linked list node and its node member functions          *********
 *******************************************************************************
 ******************************************************************************/

_llnode::_llnode() : _prev(0), _next(0), data() {};

_llnode::_llnode(volatile TaskEntry &arg, volatile _llnode *pre,
                 volatile _llnode *nex)
    : _prev(pre), _next(nex), data(arg) {};

/*******************************************************************************
 *******************************************************************************
 *********        LinkedList and its node member functions             *********
 *******************************************************************************
 ******************************************************************************/

LinkedList::LinkedList() : head(0), tail(0), size(0) {}

/**
 * Add argument into the linked list by keeping the list sorted. List sorted in
 * an ascending order by the TaskEntry._timestamp parameter. Essentially tasks
 * that need to executed sooner are at the beginning of the list.
 * @note If new task has same _timestamp value (time to be executed at) as the
 * task already in the list, new task is placed after the existing one
 * @param arg task to add to the list
 * @return pointer to the instance of task inside the list
 */
volatile _llnode* LinkedList::AddSort(TaskEntry &arg) volatile
{
    volatile _llnode *tmp = new _llnode(arg),//  Create new node on the free store
             *node = head;           //  Define starting node

    //  Find where to insert new node(worst-case: end of the list)
    while (node != 0)
    {
        //  Sorting logic - sorts list ascending, in case two tasks have the same
        //  time to be executed at, new task is added after the old on in the list
        if (tmp->data._timestamp < node->data._timestamp) break;
        //  If sorting logic doesn't break the loop move to next element
        node = node->_next;
    }

    //  Increase size of complete list
    size++;
    //*************************************************INSERTION LOGIC**/
    //  a) Haven't  moved from start - we have new smallest node
    if (node == head)   //  Insert before first element
    {
        //  Check if there're any elements at all in the list
        if (head != 0)    // If yes update head_previous node
            head->_prev = tmp;
        else                    // If not update tail node as well
            tail = tmp;

        tmp->_next = head;  // Update next node (tmp will become head node)
        head = tmp;         // Assign new head node
        return head;
    }
    // b) Reached end of the list - insert node after last one
    else if (node == 0)   //  Insert after last element
    {
        tail->_next = tmp;  // Update next element of current tail
        tmp->_prev = tail;  // Update prev element of new tail element
        tail = tmp;         // Assign new tail element
        return tail;
    }
    // c) Inserting element in the middle, BEFORE some 'node'
    else
    {
        tmp->_prev = node->_prev;
        node->_prev->_next = tmp;
        tmp->_next = node;
        node->_prev = tmp;
        return tmp;
    }
}

/**
 * Check whether the linked list is empty
 * @return true: list is empty
 *        false: list contains data
 */
bool LinkedList::Empty() volatile
{
    return (head == tail) && (head == 0);
}

/**
 * Delete content of the list.
 * Traverses all nodes in the list and erases them from free store.
 */
void LinkedList::Drop() volatile
{
    //  Check if list is already empty
    if (LinkedList::Empty()) return;

    //  Delete node by node
    while (head != 0)
    {
        //  When there's only 1 node left tail has to be manually cleared
        //  since we're clearing from head ->to-> tail
        if (head == tail)
            tail = 0;
        //  Move to next node before deleting
        volatile _llnode *tmp = head->_next;
        //  Delete head node
        delete head;
        //  Update head node
        head = tmp;
        //  Decrease size of complete list
        size--;
    }
}

/**
 * Delete first element of the list and return its ->data content
 * @return ->data content of the first node of the list
 */
TaskEntry LinkedList::PopFront() volatile
{
    //  Check if list is empty
    if (LinkedList::Empty()) return nullNode;
    //  Check if there's only one element in this list
    if (head == tail) tail = 0;
    //  Extract data from node before it's deleted
    TaskEntry retVal = head->data;
    //  Move second node to the first position
    //  If there's only one node next points to nullptr so it's safe
    volatile _llnode *newHead = head->_next;
    //  Delete data from free store
    delete head;
    //  Assign new head node
    head = newHead;
    //  Check if head can be dereferenced (!=nullptr)
    if (head != 0)
        head->_prev = 0;
    //  Decrease size of complete list
    size--;
    //  Return value stored in head node
    return retVal;
}

/**
 * Returns reference to the ->data content of first element of the list but it
 * remains in the list (it's not deleted as with PopFront)
 * @return reference to ->data content of first object of the list
 */
volatile TaskEntry& LinkedList::PeekFront() volatile
{
    return head->data;
}

/*******************************************************************************
 *******************************************************************************
 *********  	       TaskScheduler class member functions	    	   *********
 *******************************************************************************
 ******************************************************************************/

///-----------------------------------------------------------------------------
///                      Class constructor & destructor                [PUBLIC]
///-----------------------------------------------------------------------------
TaskScheduler::TaskScheduler() : _lastIndex(0)
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
 * @param time time-stamp at which to execute the task
 */
void TaskScheduler::SyncTask(uint8_t libuid, uint8_t comm, int64_t time) volatile
{
    /*
     * If time is a positive number it represent time in milliseconds from
     * start-up of the microcontroller. If time is a negative number or 0 it
     * represents a time in milliseconds from current time as provided by SysTick
     */
    if (time < 0)
        time = (uint32_t)(-time) + msSinceStartup;
    else
        time = (uint32_t)time;

    //  Save pointer to newly added task so additional arguments can be appended
    //  to it through AddArgs function call
    TaskEntry teTemp(libuid, comm, time);
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

            // Transfer data for task into kernel memory space
            __kernelVector[tE._libuid]->serviceID = tE._task;
            __kernelVector[tE._libuid]->args[0] = tE._argN;
            memcpy( (void*)(__kernelVector[tE._libuid]->args+1),
                    (void*)(tE._args),
                    tE._argN);
#if defined(__DEBUG_SESSION__)
            UARTprintf("Now is %d \n", msSinceStartup);

            UARTprintf("Processing %d:%d at %ul ms\n", tE._libuid, tE._task, tE._timestamp);
            UARTprintf("-(%d)> %s\n", tE._argN, tE._args);
#endif
            // Call kernel module to execute task
            __kernelVector[tE._libuid]->callBackFunc();
        }
}
