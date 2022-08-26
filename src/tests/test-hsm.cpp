/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Author: Matteo Piazzolla
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

#include <assert.h>
#include <events/Event.h>
#include <events/EventBroker.h>
#include <events/HSM.h>

#include <iostream>

#include "Singleton.h"

using namespace Boardcore;
using namespace std;
using namespace miosix;

// linter off

#define TOPIC_TEST 1

#define CHECK_INIT() bool testValue __attribute__((unused)) = false;

#define CHECK_STATE(HSM, SIGNAL, STATE)                            \
    do                                                             \
    {                                                              \
        cout << "------------------------------" << endl;          \
        cout << "Triggering signal " << #SIGNAL << endl;           \
        EventBroker::getInstance().post({SIGNAL}, TOPIC_TEST);     \
        Thread::sleep(400);                                        \
        testValue = HSM.testState(STATE);                          \
        cout << "Check State " << #STATE << " "                    \
             << (HSM.testState(STATE) ? "TRUE" : "FALSE") << endl; \
        assert(HSM.testState(STATE));                              \
    } while (0)

enum TestEvents : uint8_t
{
    EV_A = EV_FIRST_CUSTOM,
    EV_B,
    EV_C,
    EV_D,
    EV_E,
    EV_F,
    EV_G,
    EV_H,
    EV_I
};

class HSMUTTest : public HSM<HSMUTTest>, public Singleton<HSMUTTest>
{
    friend class Singleton<HSMUTTest>;

public:
    HSMUTTest();
    ~HSMUTTest() {}

    State state_initialization(const Event& e);
    State state_S(const Event& e);
    State state_S1(const Event& e);
    State state_S11(const Event& e);
    State state_S2(const Event& e);
    State state_S21(const Event& e);
    State state_S211(const Event& e);

    bool foo;
};

#define DEBUG_PRINT cout << __func__ << ": event received:" << (int)e << endl

using namespace std;

HSMUTTest::HSMUTTest() : HSM(&HSMUTTest::state_initialization)
{
    EventBroker::getInstance().subscribe(this, TOPIC_TEST);
}

State HSMUTTest::state_initialization(const Event& e)
{
    DEBUG_PRINT;
    this->foo = false;
    return transition(&HSMUTTest::state_S2);
}

State HSMUTTest::state_S(const Event& e)
{
    State retState = HANDLED;
    DEBUG_PRINT;
    switch (e)
    {
        case EV_ENTRY:
            break;
        case EV_INIT:
            retState = transition(&HSMUTTest::state_S1);
            break;
        case EV_EXIT:
            break;
        case EV_I:
            if (this->foo == false)
            {
                this->foo = true;
            }
            else
            {
                retState = UNHANDLED;
            }
            break;
        case EV_E:
            retState = transition(&HSMUTTest::state_S11);
            break;
        default:
            retState = tranSuper(&HSMUTTest::state_top);
            break;
    }
    return retState;
}

State HSMUTTest::state_S1(const Event& e)
{
    State retState = HANDLED;
    DEBUG_PRINT;
    switch (e)
    {
        case EV_ENTRY:
            break;
        case EV_INIT:
            retState = transition(&HSMUTTest::state_S11);
            break;
        case EV_EXIT:
            break;
        case EV_D:
            if (this->foo == false)
            {
                this->foo = true;
                retState  = transition(&HSMUTTest::state_S);
            }
            else
            {
                retState = UNHANDLED;
            }
            break;
        case EV_A:
            retState = transition(&HSMUTTest::state_S1);
            break;
        case EV_B:
            retState = transition(&HSMUTTest::state_S11);
            break;
        case EV_C:
            retState = transition(&HSMUTTest::state_S2);
            break;
        case EV_F:
            retState = transition(&HSMUTTest::state_S211);
            break;
        case EV_I:
            retState = HANDLED;
            break;
        default:
            retState = tranSuper(&HSMUTTest::state_S);
            break;
    }
    return retState;
}

State HSMUTTest::state_S11(const Event& e)
{
    State retState = HANDLED;
    DEBUG_PRINT;
    switch (e)
    {
        case EV_ENTRY:
            break;
        case EV_INIT:
            break;
        case EV_EXIT:
            break;
        case EV_D:
            if (this->foo == true)
            {
                this->foo = false;
                retState  = transition(&HSMUTTest::state_S1);
            }
            else
            {
                retState = UNHANDLED;
            }
            break;
        case EV_G:
            retState = transition(&HSMUTTest::state_S211);
            break;
        case EV_H:
            retState = transition(&HSMUTTest::state_S);
            break;
        default:
            retState = tranSuper(&HSMUTTest::state_S1);
            break;
    }
    return retState;
}

State HSMUTTest::state_S2(const Event& e)
{
    State retState = HANDLED;
    DEBUG_PRINT;
    switch (e)
    {
        case EV_ENTRY:
            break;
        case EV_INIT:
            retState = transition(&HSMUTTest::state_S211);
            break;
        case EV_EXIT:
            break;
        case EV_C:
            retState = transition(&HSMUTTest::state_S1);
            break;
        case EV_F:
            retState = transition(&HSMUTTest::state_S11);
            break;
        case EV_I:
            if (this->foo == false)
            {
                this->foo = true;
            }
            else
            {
                retState = UNHANDLED;
            }
            break;
        default:
            retState = tranSuper(&HSMUTTest::state_S);
            break;
    }
    return retState;
}

State HSMUTTest::state_S21(const Event& e)
{
    State retState = HANDLED;
    DEBUG_PRINT;
    switch (e)
    {
        case EV_ENTRY:
            break;
        case EV_INIT:
            retState = transition(&HSMUTTest::state_S211);
            break;
        case EV_EXIT:
            break;
        case EV_A:
            retState = transition(&HSMUTTest::state_S21);
            break;
        case EV_G:
            retState = transition(&HSMUTTest::state_S11);
            break;
        default:
            retState = tranSuper(&HSMUTTest::state_S2);
            break;
    }
    return retState;
}
State HSMUTTest::state_S211(const Event& e)
{
    State retState = HANDLED;
    DEBUG_PRINT;
    switch (e)
    {
        case EV_ENTRY:
            break;
        case EV_INIT:
            break;
        case EV_EXIT:
            break;
        case EV_D:
            retState = transition(&HSMUTTest::state_S21);
            break;
        case EV_H:
            retState = transition(&HSMUTTest::state_S);
            break;
        default:
            retState = tranSuper(&HSMUTTest::state_S21);
            break;
    }
    return retState;
}

/*
 * ************ END STATE MACHINE ************
 **/

int main()
{

    EventBroker::getInstance().start();

    HSMUTTest& hsm = HSMUTTest::getInstance();
    hsm.start();

    CHECK_INIT();
    // cppcheck-suppress assertWithSideEffect
    CHECK_STATE(hsm, EV_G, &HSMUTTest::state_S11);
    // cppcheck-suppress assertWithSideEffect
    CHECK_STATE(hsm, EV_I, &HSMUTTest::state_S11);
    // cppcheck-suppress assertWithSideEffect
    CHECK_STATE(hsm, EV_A, &HSMUTTest::state_S11);
    // cppcheck-suppress assertWithSideEffect
    CHECK_STATE(hsm, EV_D, &HSMUTTest::state_S11);
    // cppcheck-suppress assertWithSideEffect
    CHECK_STATE(hsm, EV_D, &HSMUTTest::state_S11);
    // cppcheck-suppress assertWithSideEffect
    CHECK_STATE(hsm, EV_B, &HSMUTTest::state_S11);
    // cppcheck-suppress assertWithSideEffect
    CHECK_STATE(hsm, EV_C, &HSMUTTest::state_S211);
    // cppcheck-suppress assertWithSideEffect
    CHECK_STATE(hsm, EV_E, &HSMUTTest::state_S11);
    // cppcheck-suppress assertWithSideEffect
    CHECK_STATE(hsm, EV_E, &HSMUTTest::state_S11);
    // cppcheck-suppress assertWithSideEffect
    CHECK_STATE(hsm, EV_G, &HSMUTTest::state_S211);
    // cppcheck-suppress assertWithSideEffect
    CHECK_STATE(hsm, EV_B, &HSMUTTest::state_S211);
    // cppcheck-suppress assertWithSideEffect
    CHECK_STATE(hsm, EV_I, &HSMUTTest::state_S211);
    // cppcheck-suppress assertWithSideEffect
    CHECK_STATE(hsm, EV_I, &HSMUTTest::state_S211);

    cout << "Test Passed!" << endl;
    while (1)
    {
    };
    return 0;
}
