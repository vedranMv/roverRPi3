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
#define TS_TASK_MAX     10
//#define __DEBUG_SESSION__
/*
class _taskEntry;
class LinkedList;
class TaskScheduler;*/

/**
 * _taksEntry class - object wrapper for tasks handled by TaskScheduler class
 */
class _taskEntry
{
    // Functions & classes needing direct access to all members
	friend class TaskScheduler;
	friend void TSSyncCallback(void);
    friend void TS_GlobalCheck(void);
    friend class LinkedList;

	public:
		_taskEntry();
		_taskEntry(const _taskEntry& arg);
		_taskEntry(const volatile _taskEntry& arg);
		_taskEntry(uint8_t uid, uint8_t task, uint32_t utcTime)
		    :_libuid(uid), _task(task), _timestamp(utcTime) {};

		void        AddArg(void* arg, uint8_t argLen);
		void        AddArg(void* arg, uint8_t argLen) volatile;
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
		//volatile TE = const TE&
        volatile _taskEntry& operator= (const volatile _taskEntry& arg)
        {
            _libuid = arg._libuid;
            _task = arg._task;
            _argN = arg._argN;
            _timestamp = arg._timestamp;

            for (uint8_t i = 0; i < sizeof(_args); i++)
                _args[i] = arg._args[i];
            return *this;
        }

        volatile _taskEntry& operator= (volatile _taskEntry& arg) volatile
        {
            _libuid = arg._libuid;
            _task = arg._task;
            _argN = arg._argN;
            _timestamp = arg._timestamp;

            for (uint8_t i = 0; i < sizeof(_args); i++)
                _args[i] = arg._args[i];
            return (volatile _taskEntry&) *this;
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

class _llnode
{
    public:
        _llnode() : _prev(0), _next(0), data() {};
        _llnode(volatile _taskEntry arg, volatile _llnode *pre = 0, volatile _llnode *nex = 0)
            : _prev(pre), _next(nex), data(arg) {};

        volatile _llnode     *_prev,
                    *_next;
        volatile _taskEntry  data;
};

class LinkedList
{
    public:
        LinkedList() : head(0), tail(0), size(0) {}

        volatile _llnode* AddSort(_taskEntry arg) volatile
        {
            volatile _llnode *tmp = new _llnode(arg),//  Create new node on the free store
                     *node = head;           //  Define starting node

            //  Find where to insert new node(worst-case: end of the list)
            while (node != 0)
            {
                //  Sorting logic
                if (tmp->data._timestamp <= node->data._timestamp) break;
                //  If sorting logic doesn't break the loop move to next element
                node = node->_next;
            }

            //  Increase size of complete list
            size++;
            /*************************************************INSERTION LOGIC**/
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

        bool Empty() volatile
        {
            return (head == tail) && (head == 0);
        }

        void Drop() volatile
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

         _taskEntry PopFront() volatile
        {
            //  Check if list is empty
            if (LinkedList::Empty()) return nullNode;
            //  Check if there's only one element in this list
            if (head == tail) tail = 0;
            //  Extract data from node before it's deleted
            _taskEntry retVal = head->data;
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

        volatile _taskEntry& PeekFront() volatile
        {
            return head->data;
        }

    public:
        volatile _llnode     *head,
                             *tail;
        volatile _taskEntry  nullNode;
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

		bool    operator()(const _taskEntry& arg1, const _taskEntry& arg2);
		void 				 Reset() volatile;
		bool				 IsEmpty() volatile;
		uint8_t              SyncTask(uint8_t libuid, uint8_t comm,
		                                       int64_t time) volatile;
		uint8_t              SyncTask(_taskEntry te) volatile;
		void                 AddStringArg(void* arg, uint8_t argLen);
		void                 AddStringArg(void* arg, uint8_t argLen) volatile;
		 _taskEntry  PopFront() volatile;
		volatile _taskEntry&  PeekFront() volatile;
		//_taskEntry           At(uint16_t index) volatile;

	private:
		//  Queue of tasks to be executed
		volatile LinkedList	_taskLog;
		//  Iterators for task queue (B-begin, E-end)
		volatile _llnode    *_lastIndex;
};

/*	Global pointer to first instance of TaskScheduler object	*/
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
