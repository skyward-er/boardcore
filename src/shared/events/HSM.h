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

// This work is derived from https://www.state-machine.com by Miro Samek.

#pragma once

#include <assert.h>
#include <events/Event.h>
#include <events/EventHandler.h>
#include <utils/Debug.h>
#include <utils/collections/SyncQueue.h>

#include "ActiveObject.h"

namespace Boardcore
{

static constexpr int8_t HSM_MAX_NEST_DEPTH = 5;

enum State
{
    HANDLED = 0,  ///< Event handled.
    UNHANDLED,    ///< Event unhandled
    IGNORED,
    TRAN,   ///< A transition to another state was taken.
    SUPER,  ///< A transition to a parent state was taken.
};

template <class T>
class HSM : public EventHandler
{
public:
    typedef State (T::*StateHandler)(const Event&);

    /**
     * Constructor.
     *
     * @param initialState Initial state of the machine.
     */
    HSM(StateHandler initialState,
        unsigned int stacksize    = miosix::STACK_DEFAULT_FOR_PTHREAD,
        miosix::Priority priority = miosix::MAIN_PRIORITY)
        : EventHandler(stacksize, priority)
    {
        state = &T::state_top;
        temp  = initialState;
    }

    virtual ~HSM(){};

    bool start() override
    {
        init();

        return EventHandler::start();
    }

    /**
     * @brief Performs a transition to the specified state.
     *
     * @param nextState Target state.
     */
    State transition(StateHandler nextState)
    {
        temp = nextState;
        return TRAN;
    }

    /**
     * @brief Performs a transaction to the specified super state.
     *
     * @param superState Target super state.
     */
    State tranSuper(StateHandler superState)
    {
        temp = superState;
        return SUPER;
    }

    /**
     * @brief Test if the state machine is in the specified state.
     *
     * @param testState State to test.
     * @return True if the state machine is in the given state.
     */
    bool testState(StateHandler testState) { return this->state == testState; }

    // Internal pointers representing the state o the HSM. If state == temp,
    // then "the configuration is stable".
    StateHandler state;
    StateHandler temp;

protected:
    /**
     * @brief Makes the current state handle the event and changes the state
     * accordingly.
     *
     * @param event The event to handle.
     */
    void handleEvent(const Event& event)
    {
        // NOTE: here state == temp

        StateHandler target = this->state;  // Current state
        StateHandler source;
        State retState;

        // Process the event hierarchically until it is either HANDLED, IGNORED
        // or TRAN
        do
        {
            // Save the current state that will handled the event
            source = this->temp;

            // Forward the event to the current state
            retState = (static_cast<T*>(this)->*source)(event);

            // If the event was unhandled by the current state, forward the
            // event to the parent state. EV_EMPTY will result in a transition
            // to the direct parent state, this temp will change
            if (retState == UNHANDLED)
                retState = (static_cast<T*>(this)->*source)({EV_EMPTY});

            // Here the return state won't be UNHANDLED. If the return state is
            // SUPER then it means that the event triggered a state change
            // upwards in the hierarchy. This means that the event has to be
            // handled by the parent, thus the loop continues until the event is
            // either HANDLED, IGNORED or TRAN

            // If the return value is TRAN, then it means that the event was
            // handled and a transition to another state occurred. Thus temp
            // will contain a new value

            // If the return value is SUPER, then temp will contain the parent
            // state that will handle the event
        } while (retState == SUPER);

        // NOTE: Here temp could be unchanged or could contain a new state that
        // the machine transitioned to. If this is the case then retState will
        // be TRAN. Otherwise the current state doesn't need to be changed

        // If the event triggered a transition to another state, then the state
        // need to be changed. If this is the case then temp contains the new
        // state to transition to
        if (retState == TRAN)
        {
            StateHandler path[HSM_MAX_NEST_DEPTH];
            int8_t index     = -1;
            int8_t tempIndex = 0;

            // NOTE: temp contains the new state to transition to
            // NOTE: target is the initial state

            // The order is such that the new state is further in the path
            path[0] = this->temp;  // Next state
            path[1] = target;      // Initial state

            // NOTE: source contains the state that handled the event and that
            // triggered the state change

            // Exit from every state from the initial state until the one that
            // handled the event. The loop stops before exiting the source
            // state (the one that handled the event)
            while (target != source)
            {
                // Exit from the current target state. The target starts from
                // the initial state

                // If the state exit is handled, then find the super state
                if ((static_cast<T*>(this)->*target)({EV_EXIT}) == HANDLED)
                    (void)(static_cast<T*>(this)->*target)({EV_EMPTY});

                target = this->temp;
                // Now target is the super state of the previous target
            }

            // NOTE: At this point EV_EXIT has been called on the initial state
            // and on every super state until the state that handled the event

            // Save in target the next state
            target = path[0];

            // NOTE: At this point:
            //   target is the state to transition to
            //   source is the state that handled the event

            // (A) If the source (state that handled the event) and the target
            // (the next state) are the same, then this is a transition to
            // itself
            if (source == target)  // source == target
            {
                // Exit the source state
                (static_cast<T*>(this)->*source)({EV_EXIT});

                // Refer to the next state
                index = 0;
            }
            else
            {
                // Otherwise find the superstate of the target (the next state)
                (static_cast<T*>(this)->*target)({EV_EMPTY});

                // The superstate now becomes the target
                target = this->temp;

                // (B) Check if the superstate of the target is the state that
                // handled the event
                if (source == target)  // source = target->super
                {
                    // Enter the target
                    index = 0;
                }
                else
                {
                    // Find the superstate of the source
                    (static_cast<T*>(this)->*source)({EV_EMPTY});

                    // (C) Check if the superstate of the source is the
                    // superstate of the target
                    if (this->temp == target)  // source->super == target->super
                    {
                        // Exit the source and enter the target
                        (static_cast<T*>(this)->*source)({EV_EXIT});
                        index = 0;
                    }
                    // (D) Check if the superstate of the source is the target
                    else if (this->temp == path[0])  // source->super = target
                    {
                        // Exit the source
                        (static_cast<T*>(this)->*source)({EV_EXIT});
                    }
                    // (E) Check the rest
                    else  // source == target->...->super
                    {
                        // Store the path along the way

                        tempIndex = 0;
                        index     = 1;

                        path[1] = target;

                        // path[0] is target
                        // path[1] is target->super

                        // Save source->super
                        target = this->temp;

                        // Find target->super->super
                        retState =
                            (static_cast<T*>(this)->*path[1])({EV_EMPTY});

                        // EV_EMPTY will result in SUPER unless state_top is
                        // reached
                        while (retState == SUPER)
                        {
                            index++;

                            // Store the current state
                            path[index] = this->temp;

                            // Is it the source?
                            if (this->temp == source)
                            {
                                // LCa (leas common ancestor) found
                                tempIndex = 1;

                                // Path must not overflow
                                D(assert(index < (int8_t)HSM_MAX_NEST_DEPTH));

                                // Do not enter the source and terminate
                                index--;
                                retState = HANDLED;
                            }
                            else
                            {
                                // It is not the source, keep going up
                                retState =
                                    (static_cast<T*>(this)->*temp)({EV_EMPTY});
                            }
                        }

                        // Here path will contain the path from the target to
                        // the source

                        if (tempIndex == 0)
                        {
                            // the LCA not found yet?

                            // Entry path must not overflow
                            D(assert(index < HSM_MAX_NEST_DEPTH));

                            // Exit the source
                            (static_cast<T*>(this)->*source)({EV_EXIT});

                            // (F) Check the rest of source->super ==
                            // target->super->super...

                            // Show that LCA was not found
                            tempIndex = index;
                            retState  = IGNORED;

                            do
                            {
                                // Is this the LCA?
                                if (target == path[tempIndex])
                                {
                                    // Indicate LCA was found
                                    retState = HANDLED;

                                    // Do not enter the LCA and terminate
                                    index     = tempIndex - 1;
                                    tempIndex = -1;
                                }
                                else
                                {
                                    // Try lower superstate of target
                                    tempIndex--;
                                }
                            } while (tempIndex >= 0);

                            // LCA not found yet?
                            if (retState != HANDLED)
                            {
                                // (G) check each source->super->... for each
                                // target->super...
                                retState = IGNORED;

                                do
                                {
                                    // Exit target unhandled?
                                    if ((static_cast<T*>(this)->*target)(
                                            {EV_EXIT}) == HANDLED)
                                        (static_cast<T*>(this)->*target)(
                                            {EV_EMPTY});

                                    // Set to super of target
                                    target    = this->temp;
                                    tempIndex = index;

                                    do
                                    {
                                        // Is this LCA?
                                        if (target == path[tempIndex])
                                        {
                                            // Do not enter LCA, break inner and
                                            // break outer
                                            index     = tempIndex - 1;
                                            tempIndex = -1;
                                            retState  = HANDLED;
                                        }
                                        else
                                        {
                                            tempIndex--;
                                        }
                                    } while (tempIndex >= 0);
                                } while (retState != HANDLED);
                            }
                        }
                    }
                }
            }

            // Travel the path in reverse
            for (; index >= 0; index--)
                (static_cast<T*>(this)->*path[index])({EV_ENTRY});

            // Stick the target into register and update the next state
            target     = path[0];
            this->temp = target;

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

        // Change the current active state, mark the configuration as stable
        this->state = target;
        this->temp  = target;

        // At the end, state and temp are equals. This means that on every call
        // of handleEvent, state == temp
    }

    State state_top(const Event&) { return IGNORED; }

private:
    /**
     * @brief Initializes the state machine executing ENTRY and INIT for the
     * initial state.
     */
    void init()
    {
        // NOTE: Here state = state_top != state

        StateHandler target = this->state;
        State retState;

        // NOTE: EV_EMPTY requires the state to move to the parent state with
        // tranSuper

        // At start, temp is a fictitious state and handling EV_EMPTY will
        // transition to the real initial state
        // State s __attribute__((unused)) =
        //     (static_cast<T*>(this)->*temp)({EV_EMPTY});
        // TODO: Check lines above!

        // This is to assert that the first state given to the constructor
        // transition to the initial state
        D(assert(s == TRAN));

        // NOTE: At this point, temp is the initial state

        do
        {
            StateHandler statePath[HSM_MAX_NEST_DEPTH];
            int8_t index = 0;

            // Save the current state
            statePath[0] = this->temp;

            // Transition to the next parent state
            (void)(static_cast<T*>(this)->*temp)({EV_EMPTY});

            // Find the state hierarchy from the current state to state_top.
            // This will loop until temp is hsp_top (target is initialized to
            // state_top in the constructor) or the maximum allowed depth is
            // reached
            while (this->temp != target && index < HSM_MAX_NEST_DEPTH - 1)
            {
                index++;

                // Save the current state at the current index
                statePath[index] = this->temp;

                // Transition to the next parent state
                (void)(static_cast<T*>(this)->*temp)({EV_EMPTY});

                // This is useless given the while condition
                D(assert(index < HSM_MAX_NEST_DEPTH));
            }

            // NOTE: At this point in statePath we have the state hierarchy from
            // the initial state up to state_top (or the nest limit)

            // Reset the current state to the initial state
            this->temp = statePath[0];

            // Forward EV_ENTRY to every state in the current hierarchy from the
            // top state down to the initial state
            do
            {
                (void)(static_cast<T*>(this)->*statePath[index])({EV_ENTRY});
                index--;
            } while (index >= 0);

            // Save the initial state
            target = statePath[0];

            // Forward EV_INIT to the initial state
            retState = (static_cast<T*>(this)->*temp)({EV_INIT});

            // If EV_INIT triggered a transition to another state, the whole
            // process needs to be repeated.
        } while (retState == TRAN);

        this->state = target;
        this->temp  = target;
    }
};

}  // namespace Boardcore
