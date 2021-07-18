/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Federico Terraneo, Matteo Piazzolla
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef SRC_SHARED_ACTIVEOBJECT_H
#define SRC_SHARED_ACTIVEOBJECT_H

#include <Common.h>

#include "diagnostic/SkywardStack.h"

/**
 * Utility class implementing the Active Object pattern
 * Deriving from this class causes a thread to be spawned for each
 * instance. This thread will call the run() function.
 */
class ActiveObject
{

public:
    /**
     * Constructor. This will create the AO, but will NOT start the thread
     * associated with it. call start() in order to start the thread. \param
     * stacksize the stack of the thread that will be spawned \param priority
     * priority of the thread that will be spawned
     */
    ActiveObject(unsigned int stacksize    = miosix::STACK_DEFAULT_FOR_PTHREAD,
                 miosix::Priority priority = miosix::MAIN_PRIORITY)
        : stack_size(skywardStack(stacksize)), priority(priority)
    {
    }

    virtual ~ActiveObject(){};

    /**
     * @brief Start the thread associated with this activeobject.
     * Call stop() to terminate execution of the thread.
     * @return true
     * @return false
     */
    virtual bool start()
    {
        if (!started && !stopped)
        {
            thread = miosix::Thread::create(
                threadLauncher, stack_size, priority,
                reinterpret_cast<void*>(this), miosix::Thread::JOINABLE);

            if (thread != nullptr)
                started = true;

            return started;
        }
        else
        {
            return false;
        }
    }

    /**
     * @brief Signals the runner thread to terminate and joins the thread.
     * This is a blocking call that will not return until the thread terminates!
     * Your run() implementation must check shouldStop() and terminate ASAP if
     * it returns true.
     */
    virtual void stop()
    {
        if (isRunning())
        {
            should_stop = true;
            thread->join();
            stopped = true;
        }
    }

    bool isStarted() { return started; }
    bool isStopped() { return stopped; }

    bool isRunning() { return started && !stopped; }

protected:
    /**
     * The thread that will be spawned just calls this function.
     * Override it to implement your logic.
     * Remember to frequently check for shouldStop() to see if you should
     * terminate the execution of the thread.
     */
    virtual void run() = 0;

    /**
     * @brief Tells whether or not the stop() function has been called, and
     * so if the ActiveObject should stop its execution. Your implementation
     * of the run() method should periodically check this function and
     * should terminate as soon as possible if required to.
     *
     * @return true if stop() has been called and the ActiveObject should
     * stop executing
     */
    bool shouldStop() { return should_stop; }

    miosix::Thread* thread = nullptr;  ///< Gives access to the thread object
    bool should_stop       = false;
    bool stopped           = false;

private:
    unsigned int stack_size;
    miosix::Priority priority;

    /**
     * Calls the run member function
     * \param arg the object pointer cast to void*
     */
    static void threadLauncher(void* arg)
    {
        reinterpret_cast<ActiveObject*>(arg)->run();
    }

    bool started = false;
};

#endif  // SRC_SHARED_ACTIVEOBJECT_H
