/**
 * linkedList.cpp
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran
 */
#include "linkedList.h"
#ifdef __DEBUG_SESSION__
#include "serialPort/uartHW.h"
#endif

/*******************************************************************************
  *********         Linked list node - member functions                *********
 ******************************************************************************/
_llnode::_llnode() : _prev(0), _next(0), data() {};

_llnode::_llnode(volatile TaskEntry &arg, volatile _llnode *pre,
                 volatile _llnode *nex)
    : _prev(pre), _next(nex), data(arg) {};


/*******************************************************************************
 *********          LinkedList  member functions                       *********
 ******************************************************************************/
LinkedList::LinkedList() : head(0), tail(0), size(0) {}

LinkedList::~LinkedList()
{
    //  Delete any data in the list when it goes out of scope
    if (size > 0)
        Drop();
}

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
 * Find and delete from linked list a task passed as an argument
 * @note task in arg has valid libUID, taskID and arguments
 * @param arg
 * @return true if task was found and deleted, false otherwise
 */
bool LinkedList::RemoveEntry(TaskEntry &arg) volatile
{
    volatile _llnode *node = head;           //  Define starting node

    while (node != tail)
    {
        //  Check for matching libUID
        if (node->data._libuid != arg._libuid)
        {
            node = node->_next;
            continue;
        }
        //  Check for matching taskID
        if (node->data._task != arg._task)
        {
            node = node->_next;
            continue;
        }
        //  Check for matching length of arguments
        if (node->data._argN != arg._argN)
        {
            node = node->_next;
            continue;
        }
        //  Check if arguments match
        for (uint16_t i = 0; i < arg._argN; i++)
            if (node->data._args[i] != arg._args[i])
            {
                node = node->_next;
                continue;
            }
        //  If we got to here we have a match, remove node but link _prev and
        //  _next if they exists
        if (node->_prev != 0)
            node->_prev->_next = node->_next;
        if (node->_next != 0)
            node->_next->_prev = node->_prev;
        //  Check if the node was head or tail and update those
        if (head == node)
            head = node->_next;
        if (tail == node)
            tail == node->_prev;
        delete node;
        //  Node has been found and deleted, return true
        return true;
    }

    //  Node wasn't found in the list, return false
    return false;
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
