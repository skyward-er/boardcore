/* Event scheduler
 *
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Author: Federico Terraneo, Matteo Michele Piazzolla
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

/**
 * Utility class implementing the Active Object pattern
 * Deriving from this class causes a thread to be spawned for each
 * instance. This thread will call the run() function.
 */
class ActiveObject {
    
public:
    /**
     * Constructor
     * \param stacksize the stack of the thread that will be spawned
     * \param priority priority of the thread that will be spawned
     */
    ActiveObject(unsigned int stacksize = miosix::STACK_DEFAULT_FOR_PTHREAD,
                 miosix::Priority priority = miosix::MAIN_PRIORITY)
    {
        thread=miosix::Thread::create(threadLauncher,
                              stacksize,
                              priority,
                              reinterpret_cast<void*>(this));
    }

protected:
    /**
     * The thread that will be spawned just calls this function.
     * Override it to implement your logic.
     */
    virtual void run()=0;

    miosix::Thread* thread; ///< Gives access to the thread object

private:
    /**
     * Calls the run member function
     * \param arg the object pointer cast to void*
     */
    static void threadLauncher(void* arg)
    {
        reinterpret_cast<ActiveObject*>(arg)->run();
    }
};
