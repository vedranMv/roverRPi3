/**
 * taskEntry.cpp
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran
 */
#include "taskEntry.h"


///-----------------------------------------------------------------------------
///                      Class constructors                             [PUBLIC]
///-----------------------------------------------------------------------------
TaskEntry::TaskEntry() : _libuid(0), _task(0), _argN(0), _timestamp(0), _args(0)
{
}

TaskEntry::TaskEntry(uint8_t uid, uint8_t task, uint32_t time,
                     int32_t period, int32_t repeats)
            :_libuid(uid), _task(task), _timestamp(time),
             _argN(0), _args(0), _period(period), _repeats(repeats)
{
}

TaskEntry::TaskEntry(const TaskEntry& arg) :  _argN(0), _args(0)
{
    *this = (const TaskEntry&)arg;
}

TaskEntry::TaskEntry(const volatile TaskEntry& arg) :  _argN(0), _args(0)
{
    *this = (const volatile TaskEntry&)arg;
}

TaskEntry::~TaskEntry()
{
    //  If there's any dynamically allocated data release it
    if (_args != 0)
        delete [] _args;
}

/**
 * Add argument(s) stored in a byte array [arg] of length [argLen]. Byte array
 * may contain data of any type, as long as receiver of that data knows how to
 * interpret bytes stored in the field.
 * Size of internal array holding bytes is dynamically reallocated every time
 * this function is called in order to ensure there's enough space for all args.
 * @note This function doesn't have overflow protection. It will try to save all
 * provided arguments into and array, allocating as much space as it needs.
 * @param arg byte array of data to pass to the function
 * @param argLen length of byte array [arg] (in bytes)
 */
void TaskEntry::AddArg(void* arg, uint16_t argLen) volatile
{
    //  Allocate new memory to fit all the arguments +1 space because argument
    //  array has to be null-terminated
    uint8_t *temp = new uint8_t[_argN+argLen+1];

    //  Copy existing arguments from _args into a new memory location
    memcpy((void*)temp, (void*)_args, _argN);
    //  Delete data currently stored in pointer _args
    delete [] _args;
    //  Append new arguments to the new array of arguments
    memcpy((void*)(temp+_argN), arg, argLen);
    _argN += argLen;
    //  Null-terminate array
    temp[_argN] = 0;
    //  Save new array into a pointer in this object
    _args = temp;
}

///-----------------------------------------------------------------------------
///                 Class operator definitions                          [PUBLIC]
///-----------------------------------------------------------------------------

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
    _period = arg._period;
    _repeats = arg._repeats;

    _args = new uint8_t[_argN];
    memcpy((void*)_args, (void*)(arg._args), _argN);
    return *this;
}

volatile TaskEntry& TaskEntry::operator= (const volatile TaskEntry& arg)
{
    _libuid = arg._libuid;
    _task = arg._task;
    _argN = arg._argN;
    _timestamp = arg._timestamp;
    _period = arg._period;
    _repeats = arg._repeats;

    _args = new uint8_t[_argN];
    memcpy((void*)_args, (void*)(arg._args), _argN);
    //for (uint8_t i = 0; i < _argN; i++)
     //   _args[i] = arg._args[i];
    return *this;
}

volatile TaskEntry& TaskEntry::operator= (volatile TaskEntry& arg) volatile
{
    _libuid = arg._libuid;
    _task = arg._task;
    _argN = arg._argN;
    _timestamp = arg._timestamp;
    _period = arg._period;
    _repeats = arg._repeats;

    _args = new uint8_t[_argN];
    memcpy((void*)_args, (void*)(arg._args), _argN);
    //for (uint8_t i = 0; i < _argN; i++)
    //    _args[i] = arg._args[i];
    return (volatile TaskEntry&) *this;
}
