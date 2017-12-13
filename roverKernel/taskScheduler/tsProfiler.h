/**
 *  tsProfiler.h
 *
 *  Created on: 14.11.2017.
 *      Author: Vedran Mikov
 *
 *  Task scheduler extension for profiling of tasks (measuring run-time statistics)
 *  @version 1.0
 *  V1.0
 *  +Creation of file, definition of class object for holding task-performance data
 *  V1.1
 *  +Added ability to measure average task runtime by accumulating all run times
 *  into a 32-bit counter and dividing by number of runs
 */

#ifndef ROVERKERNEL_TASKSCHEDULER_TSPROFILER_H_
#define ROVERKERNEL_TASKSCHEDULER_TSPROFILER_H_


class Performance
{
    public:
        Performance(): startTimeMissTot(0), startTimeMissCnt(0), taskRuns(0),
                       maxRT(0), _lastStartT(0), msAcc(0), accRT(0) {};
        ~Performance() {};

        void TaskStartHook(const uint64_t &timestamp,
                           const uint64_t &taskStartTime,
                           const uint64_t &timeStep)
        {
            //  If we missed starting time of the task for more than 1 time-step
            //  calculate for how much was the deadline missed and increase count
            //  of missed tasks
            if (timestamp > (taskStartTime + timeStep))
            {
                startTimeMissCnt++;
                startTimeMissTot += (uint32_t)(timestamp - taskStartTime);
            }

            //  Save timestamp for calculating execution time
            _lastStartT = timestamp;
            taskRuns++;
        }
        void TaskEndHook(const uint64_t &timestamp)
        {
            //  Calculate run-time of task once it's finished
            uint16_t rt = (uint16_t)(timestamp - _lastStartT);

            //  Check if we have new maximum run time
            if (rt > maxRT)
                maxRT = rt;

            //  Update millisecond accumulator and accumulated runtime
            accRT += (msAcc+rt) / 1000;
            msAcc = (msAcc+rt) % 1000;
        }

        //  TODO: Make sure to include all new variables in these assignments
        Performance& operator= (Performance &arg)
        {
            startTimeMissTot = arg.startTimeMissTot;
            startTimeMissCnt = arg.startTimeMissCnt;
            taskRuns = arg.taskRuns;
            maxRT = arg.maxRT;
            msAcc = arg.msAcc;
            accRT = arg.accRT;

            return *this;
        }


        Performance& operator= (const Performance &arg)
        {
            startTimeMissTot = arg.startTimeMissTot;
            startTimeMissCnt = arg.startTimeMissCnt;
            taskRuns = arg.taskRuns;
            maxRT = arg.maxRT;
            msAcc = arg.msAcc;
            accRT = arg.accRT;

            return *this;
        }

        Performance& operator= (const volatile Performance &arg)
        {
            startTimeMissTot = arg.startTimeMissTot;
            startTimeMissCnt = arg.startTimeMissCnt;
            taskRuns = arg.taskRuns;
            maxRT = arg.maxRT;
            msAcc = arg.msAcc;
            accRT = arg.accRT;

            return *this;
        }

        void operator= (const volatile Performance &arg) volatile
        {
            startTimeMissTot = arg.startTimeMissTot;
            startTimeMissCnt = arg.startTimeMissCnt;
            taskRuns = arg.taskRuns;
            maxRT = arg.maxRT;
            msAcc = arg.msAcc;
            accRT = arg.accRT;
        }

    public:
        //  Sum of time time differences between actual & specified  start time
        uint32_t startTimeMissTot;
        //  Number of times the task has missed its starting time
        uint32_t startTimeMissCnt;
        //  Number of times the task has run
        uint32_t taskRuns;
        //  Max run-time
        uint16_t maxRT;
        //  MS accumulator -> hold milliseconds until they can be transformed
        //  into a second (used only for short tasks)
        uint16_t msAcc;
        //  Accumulated task runtime in seconds
        uint32_t accRT;

    protected:
        //  Last start time of the task -> used to calculate runtime
        uint64_t _lastStartT;
};


#endif /* ROVERKERNEL_TASKSCHEDULER_TSPROFILER_H_ */
