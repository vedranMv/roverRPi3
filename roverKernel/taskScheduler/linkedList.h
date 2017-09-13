/**
 * linkedList.h
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran Mikov
 */
#ifndef ROVERKERNEL_TASKSCHEDULER_LINKEDLIST_H_
#define ROVERKERNEL_TASKSCHEDULER_LINKEDLIST_H_

#include "taskEntry.h"

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

        volatile _llnode*   AddSort(TaskEntry &arg) volatile;
        bool                RemoveEntry(TaskEntry &arg) volatile;
        bool                RemoveEntry(uint16_t PIDarg) volatile;
        bool                Drop() volatile;
        TaskEntry           PopFront() volatile;

        ///---------------------------------------------------------------------
        ///                      Inline functions                       [PUBLIC]
        ///---------------------------------------------------------------------
        /**
         * Check whether the linked list is empty
         * @return true: list is empty
         *        false: list contains data
         */
        inline bool IsEmpty() volatile
        {
            return (head == tail) && (head == 0);
        }
        /**
         * Returns reference to the ->data content of first element of the list but it
         * remains in the list (it's not deleted as with PopFront)
         * @return reference to ->data content of first object of the list
         */
        inline volatile TaskEntry& PeekFront() volatile
        {
            return head->data;
        }

    private:
        //  Volatile pointers as they might change inside ISRs
        volatile _llnode     * volatile head,
                             * volatile tail;
        const volatile TaskEntry   nullNode;
        volatile uint32_t    size;

};


#endif /* ROVERKERNEL_TASKSCHEDULER_LINKEDLIST_H_ */
