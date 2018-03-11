/* FSM Events
 *
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Author: Matteo Michele Piazzolla, Alain Carlucci
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

#ifndef FSM_H
#define FSM_H
#include "ActiveObject.h"
#include "SyncQueue.h"
#include "Event.h"


template<class T>
class FSM : ActiveObject
{


public:

	FSM(void (T::*initialState)(const Event*)) {
		state = initialState;
        specialEvent.sig=ENTRY;
		dispatchEvent(&specialEvent);
	}

    void postEvent(Event* e) {
    	eventList.put(e);
    }

	virtual ~FSM(){}
    void transition(void (T::*nextState)(const Event*)) {
    	specialEvent.sig=EXIT;
		(static_cast<T*>(this)->*state)(&specialEvent);
    	state = nextState;
    	specialEvent.sig=ENTRY;
		(static_cast<T*>(this)->*state)(&specialEvent);
    }
protected:


private:
    void (T::*state)(const Event*);
    SynchronizedQueue<Event*> eventList;
	Event specialEvent;

	void dispatchEvent(const Event* e) {
		(static_cast<T*>(this)->*state)(e);
	}

    void run() {
    	while(true){
    		const Event*  e = eventList.get();
    		dispatchEvent(e);
    	}
    }
};
#endif //FSM_H
