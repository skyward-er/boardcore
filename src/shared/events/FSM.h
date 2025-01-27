/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Matteo Piazzolla, Alain Carlucci, Luca Erbetta
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

#pragma once

#include <events/Event.h>
#include <events/EventBroker.h>
#include <events/EventHandler.h>
#include <utils/collections/SyncQueue.h>

#include <chrono>
#include <csetjmp>

#include "ActiveObject.h"

namespace Boardcore
{

template <class T>
class FSM : public EventHandler
{
public:
    FSM(void (T::*initialState)(const Event&),
        unsigned int stacksize    = miosix::STACK_DEFAULT_FOR_PTHREAD,
        miosix::Priority priority = miosix::MAIN_PRIORITY);

    virtual ~FSM();

    void transition(void (T::*nextState)(const Event&));

    /**
     * @brief Test if the FSM is in the given state.
     *
     * @param testState State to test.
     */
    bool testState(void (T::*testState)(const Event&));

protected:
    void run() override;
    void handleEvent(const Event& e) override;

    void waitEvent(Event events);
    void waitFor(std::chrono::milliseconds time);

private:
    void (T::*state)(const Event&);
    Event specialEvent;

    std::jmp_buf runLoop;

protected:
    // Async continuation support
    uint16_t asyncDelayedEventId;
    uint32_t asyncContinuation = 0;
};

template <class T>
FSM<T>::FSM(void (T::*initialState)(const Event&), unsigned int stacksize,
            miosix::Priority priority)
    : EventHandler(stacksize, priority)
{
    state        = initialState;
    specialEvent = EV_ENTRY;
    postEvent(specialEvent);
}

template <class T>
FSM<T>::~FSM(){};

template <class T>
void FSM<T>::transition(void (T::*nextState)(const Event&))
{
    specialEvent = EV_EXIT;
    (static_cast<T*>(this)->*state)(specialEvent);
    state        = nextState;
    specialEvent = EV_ENTRY;
    (static_cast<T*>(this)->*state)(specialEvent);

    // TODO: explain
    longjmp(runLoop, 1);
}

template <class T>
bool FSM<T>::testState(void (T::*testState)(const Event&))
{
    return (this->state == testState);
}

template <class T>
void FSM<T>::run()
{
    // TODO: explain
    setjmp(runLoop);

    while (!shouldStop())
    {
        Event e = eventList.get();
        handleEvent(e);
    }
}

template <class T>
void FSM<T>::handleEvent(const Event& e)
{
    (static_cast<T*>(this)->*state)(e);
}

template <class T>
void FSM<T>::waitEvent(Event event)
{
    while (!shouldStop())
    {
        Event e = eventList.get();

        if (e == event)
            // The event we were waiting for has arrived
            return;

        handleEvent(e);
    }
}

template <class T>
void FSM<T>::waitFor(std::chrono::milliseconds time)
{
    EventBroker::getInstance().postDelayed(EV_ASYNC_CONTINUE, 0, time.count());
    waitEvent(EV_ASYNC_CONTINUE);
}

/**
 * @brief Asynchronous continuation support.
 *
 * The following macros provide support for asynchronous continuation inside a
 * single state handler. This allows to write code that will be executed
 * asynchronously just like synchronous code, without the need to split the
 * logic into multiple states or switch-case branches.
 *
 * This enables stateful algorithms to be written in a more linear and readable
 * way.
 *
 * @example
 *  void MyFSM::state_myState(const Event& event) {
 *      ASYNC_BEGIN(TOPIC_MY_FSM);
 *      ...some operation...
 *      ASYNC_WAIT_FOR(1000);
 *
 *      // Code executed after 1 second
 *      ...some other operation...
 *      ASYNC_WAIT_FOR(2000);
 *
 *      // Code executed after 2 seconds
 *     ...some other operation...
 *      ASYNC_END();
 *  }
 */

/**
 * @brief Begins an asynchronous continuation context.
 *
 * This macro defines the beginning of a region of code that will be executed
 * asynchronously. When a wait point is reached, the FSM will return from the
 * state handler and resume from the waiting point at a later time.
 *
 * @note This macro must be followed by an ASYNC_END() macro.
 * @note Local variables are not preserved between wait points, so they must be
 * declared as class members.
 * @note All macros assume that the name of the event parameter is "event".
 *
 * @param _topic Topic to use for the async events. Must be unique for each FSM.
 */
#define ASYNC_BEGIN(_topic)                                            \
    constexpr auto _ASYNC_TOPIC = _topic;                              \
    /* Reset async continuation if entering the state */               \
    if (event == EV_ENTRY)                                             \
        asyncContinuation = 0;                                         \
    /* Remove any pending async event if exiting the state */          \
    if (event == EV_EXIT)                                              \
        EventBroker::getInstance().removeDelayed(asyncDelayedEventId); \
                                                                       \
    if (event == EV_ASYNC_CONTINUE || event == EV_ENTRY)               \
    {                                                                  \
        switch (asyncContinuation)                                     \
        {                                                              \
            case 0:

/**
 * @brief Ends an asynchronous continuation context.
 *
 * This macro defines the end of an asynchronous continuation context.
 */
#define ASYNC_END() \
    } /* switch */  \
    return;         \
    } /* if */

/**
 * @brief Waits for the specified amount of time before continuing execution.
 *
 * This macro defines a wait point in an asynchronous continuation context.
 * When this point is reached, the FSM will post a delayed EV_ASYNC_CONTINUE
 * event of the specified time and return from the state handler. When the
 * delayed event is finally received, the FSM will resume execution from the
 * waiting point.
 */
#define ASYNC_WAIT_FOR(t)                                                 \
    case __LINE__:                                                        \
        if (asyncContinuation != __LINE__)                                \
        {                                                                 \
            asyncContinuation   = __LINE__;                               \
            asyncDelayedEventId = EventBroker::getInstance().postDelayed( \
                EV_ASYNC_CONTINUE, _ASYNC_TOPIC, t);                      \
            return;                                                       \
        }

}  // namespace Boardcore
