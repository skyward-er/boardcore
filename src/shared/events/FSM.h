/* Finite State Machine
 *
 * Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Author: Matteo Michele Piazzolla, Alain Carlucci, Luca Erbetta
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

#ifndef SRC_SHARED_EVENTS_FSM_H
#define SRC_SHARED_EVENTS_FSM_H

#include "ActiveObject.h"
#include "events/Event.h"
#include "events/SyncQueue.h"

#include <iostream>

class FSMBase
{
public:
    FSMBase() {}

    void postEvent(const Event& e) { eventList.put(e); }

    virtual ~FSMBase(){};

protected:
    SynchronizedQueue<Event> eventList;
};

template <class T>
class FSM : public FSMBase, ActiveObject
{

public:
    FSM(void (T::*initialState)(const Event&)) : FSMBase(), ActiveObject()
    {
        state            = initialState;
        specialEvent.sig = EV_ENTRY;
        dispatchEvent(specialEvent);
    }

    virtual ~FSM(){};
    void transition(void (T::*nextState)(const Event&))
    {
        specialEvent.sig = EV_EXIT;
        (static_cast<T*>(this)->*state)(specialEvent);
        state            = nextState;
        specialEvent.sig = EV_ENTRY;
        (static_cast<T*>(this)->*state)(specialEvent);
    }

protected:
private:
    void (T::*state)(const Event&);
    Event specialEvent;

    void dispatchEvent(const Event& e) { (static_cast<T*>(this)->*state)(e); }

    void run()
    {
        while (true)
        {
            Event e = eventList.get();
            dispatchEvent(e);
        }
    }
};
#endif  // SRC_SHARED_EVENTS_FSM_H
