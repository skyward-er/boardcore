/* Hierarchical State Machine
 *
 * Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Author: Matteo Michele Piazzolla, Alain Carlucci, Luca Erbetta
 *
 * This work is derived from https://www.state-machine.com by Miro Samek.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated docuthisntation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, thisrge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF thisRCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEthisNT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef SRC_SHARED_EVENTS_HSM_H
#define SRC_SHARED_EVENTS_HSM_H

#include "ActiveObject.h"
#include "Debug.h"
#include "events/Event.h"
#include "events/EventHandler.h"
#include "events/SyncQueue.h"

#include <assert.h>

#define HSM_MAX_NEST_DEPTH 5

enum State
{
    HANDLED   = 0,
    IGNORED   = 1,
    TRAN      = 2,
    SUPER     = 3,
    UNHANDLED = 4
};

template <class T>
class HSM : public EventHandler
{

public:
    /**
     * Constructor
     * @param initialState func ptr of initial state in the state machine
     */
    HSM(State (T::*initialState)(const Event&)) : EventHandler()
    {
        state = &T::Hsm_top;
        temp  = initialState;
    }

    virtual ~HSM(){};

    void start()
    {
        init();

        EventHandler::start();
    }

    /**
     * Performs a transiction to the next state
     * @param nextState target state
     */
    State transition(State (T::*nextState)(const Event&))
    {
        temp = nextState;
        return TRAN;
    }

    /**
     * Performs a transiction to the super state
     * @param superState super state
     */
    State tran_super(State (T::*superState)(const Event&))
    {
        temp = superState;
        return SUPER;
    }

    /**
     * Test if the HSM is in a state
     * @param test_state state to test
     */
    bool testState(State (T::*test_state)(const Event&))
    {
        return (this->state == test_state);
    }

    /* Internal pointers representing the state o the HSM*/
    State (T::*state)(const Event&);
    State (T::*temp)(const Event&);

protected:
    /**
     * Overridden function, just handle the event
     * calling the state ptr function
     * @param e the event
     */
    void handleEvent(const Event& e)
    {
        typedef State (T::*StateHandler)(const Event&);

        StateHandler target = this->state;
        StateHandler source;
        State retState;

        /* process the event hierarchically... */
        do
        {
            source = this->temp;
            /* invoke state handler s */
            retState = (static_cast<T*>(this)->*source)(e);

            /* if unhandled due to a guard, then find superstate of s */
            if (retState == UNHANDLED)
            {
                retState = (static_cast<T*>(this)->*source)({EV_EMPTY});
            }
        } while (retState == SUPER);

        /* transition taken! */
        if (retState == TRAN)
        {
            StateHandler path[HSM_MAX_NEST_DEPTH];
            /* transition entry path index and helper transition entry path
             * index*/
            int8_t index     = (int8_t)(-1);
            int8_t tempIndex = 0;

            /* save the target of the transition */
            path[0] = this->temp;
            path[1] = target;

            /* exit current state to transition source s... */
            while (target != source)
            {
                /* if exit handled then find superstate of t */
                if ((static_cast<T*>(this)->*target)({EV_EXIT}) == HANDLED)
                {
                    (void)(static_cast<T*>(this)->*target)({EV_EMPTY});
                }

                /* this->temp holds the superstate */
                target = this->temp;
            }

            target = path[0];

            /* (A) check source==target (transition to self) */
            if (source == target)
            {
                /* exit the source and enter the target */
                (static_cast<T*>(this)->*source)({EV_EXIT});
                index = 0;
            }
            else
            {
                /* Superstate of target */
                (static_cast<T*>(this)->*target)({EV_EMPTY});
                target = this->temp;

                /* (B) check source==target->super  enter the target */
                if (source == target)
                {
                    index = (int8_t)0;
                }
                else
                {
                    /* superstate of source */
                    (static_cast<T*>(this)->*source)({EV_EMPTY});

                    /* (C) check source->super==target->super, exit the source
                     * and enter the target */
                    if (this->temp == target)
                    {
                        (static_cast<T*>(this)->*source)({EV_EXIT});
                        index = (int8_t)0;
                    }
                    else
                    {
                        /* (D) check source->super==target, exit the source */
                        if (this->temp == path[0])
                        {
                            (static_cast<T*>(this)->*source)({EV_EXIT});
                        }
                        else
                        {
                            /* (E) check rest of source==target->super->super..
                             *and store the entry path along the way
                             ** indicate that LCA not found, enter target and
                             *its superstate
                             ** save the superstate of target */
                            tempIndex = (int8_t)0;
                            index     = (int8_t)1;
                            path[1]   = target;

                            /* save source->super */
                            target = this->temp;
                            /* find target->super->super */
                            retState =
                                (static_cast<T*>(this)->*path[1])({EV_EMPTY});

                            while (retState == SUPER)
                            {
                                ++index;

                                /* store the entry path */
                                path[index] = this->temp;

                                /* is it the source? */
                                if (this->temp == source)
                                {
                                    tempIndex =
                                        (int8_t)1; /* indicate that LCA found */
                                    /* entry path must not overflow */
                                    // ASSERT(index <
                                    // (int8_t)HSM_MAX_NEST_DEPTH);

                                    /* do not enter the source and terminate the
                                     * loop */
                                    --index;
                                    retState = HANDLED;
                                }
                                else
                                { /* it is not the source, keep going up */
                                    retState = (static_cast<T*>(this)->*temp)(
                                        {EV_EMPTY});
                                }
                            }
                            if (tempIndex == (int8_t)0)
                            {
                                /* the LCA not found yet? entry path must not
                                 * overflow */
                                // ASSERT(index < (int8_t)HSM_MAX_NEST_DEPTH);
                                /* exit the source */
                                (static_cast<T*>(this)->*source)({EV_EXIT});
                                /* (F) check the rest of source->super ==
                                 * target->super->super... */
                                /* indicate LCA NOT found */
                                tempIndex = index;
                                retState  = IGNORED;
                                do
                                {
                                    /* Is this the LCA? */
                                    if (target == path[tempIndex])
                                    {
                                        /* indicate LCA found */
                                        retState = HANDLED;
                                        /*do not enter LCA and terminate the
                                         * loop */
                                        index = (int8_t)(tempIndex - (int8_t)1);
                                        tempIndex = (int8_t)(-1);
                                    }
                                    else
                                    {
                                        /* try lower superstate of target */
                                        --tempIndex;
                                    }
                                } while (tempIndex >= (int8_t)0);

                                /* LCA not found yet? */
                                if (retState != HANDLED)
                                {
                                    /* (g) check each source->super->... for
                                     * each target->super...*/
                                    /* keep looping */
                                    retState = IGNORED;
                                    do
                                    {
                                        /* exit t unhandled? */

                                        if ((static_cast<T*>(this)->*target)(
                                                {EV_EXIT}) == HANDLED)
                                        {
                                            (static_cast<T*>(this)->*target)(
                                                {EV_EMPTY});
                                        }

                                        /*  set to super of t */
                                        target    = this->temp;
                                        tempIndex = index;
                                        do
                                        {
                                            /* is this LCA? */
                                            if (target == path[tempIndex])
                                            {
                                                /* do not enter LCA, break inner
                                                 * and break outer */
                                                index     = (int8_t)(tempIndex -
                                                                 (int8_t)1);
                                                tempIndex = (int8_t)(-1);
                                                retState  = HANDLED;
                                            }
                                            else
                                            {
                                                --tempIndex;
                                            }
                                        } while (tempIndex >= (int8_t)0);
                                    } while (retState != HANDLED);
                                }
                            }
                        }
                    }
                }
            }

            /* retrace the entry path in reverse (desired) order... , enter
             * path[ip]*/
            for (; index >= (int8_t)0; --index)
            {
                (static_cast<T*>(this)->*path[index])({EV_ENTRY});
            }
            /* stick the target into register and update the next state */
            target     = path[0];
            this->temp = target; /*  */

            /* drill into the target hierarchy... */
            while ((static_cast<T*>(this)->*target)({EV_INIT}) == TRAN)
            {
                index   = (int8_t)0;
                path[0] = this->temp;

                /* find superstate */
                (static_cast<T*>(this)->*temp)({EV_EMPTY});

                while (this->temp != target)
                {
                    ++index;
                    path[index] = this->temp;
                    /*find superstate */
                    (static_cast<T*>(this)->*temp)({EV_EMPTY});
                }
                this->temp = path[0];
                /* entry path must not overflow */
                D(assert(index < (int8_t)HSM_MAX_NEST_DEPTH));

                /* retrace the entry path in reverse (correct) order... */
                do
                {
                    (static_cast<T*>(this)->*path[index])({EV_ENTRY});
                    --index;
                } while (index >= (int8_t)0);

                target = path[0];
            }
        }

        /* change the current active state, mark the configuration as stable  */
        this->state = target;
        this->temp  = target;
    }

    State Hsm_top(const Event&) { return IGNORED; }

private:
    /**
     * @brief Initializes the state machine executing ENTRY and INIT in the
     * state hierarchy
     *
     */
    void init()
    {
        typedef State (T::*StateHandler)(const Event&);
        StateHandler target = this->state;
        State retState;

        D(assert((static_cast<T*>(this)->*temp)({EV_EMPTY}) == TRAN));

        do
        {
            StateHandler statePath[HSM_MAX_NEST_DEPTH];
            int8_t index = (int8_t)0;

            statePath[0] = this->temp;
            (void)(static_cast<T*>(this)->*temp)({EV_EMPTY});

            while (this->temp != target && index < HSM_MAX_NEST_DEPTH - 1)
            {
                ++index;
                statePath[index] = this->temp;
                (void)(static_cast<T*>(this)->*temp)({EV_EMPTY});
                D(assert(index < HSM_MAX_NEST_DEPTH));
            }

            this->temp = statePath[0];

            do
            {
                (void)(static_cast<T*>(this)->*statePath[index])({EV_ENTRY});
                --index;
            } while (index >= (int8_t)0);

            target   = statePath[0];
            retState = (static_cast<T*>(this)->*temp)({EV_INIT});
        } while (retState == TRAN);

        this->state = target;
        this->temp  = target;
    }
};
#endif  // SRC_SHARED_EVENTS_HSM_H
