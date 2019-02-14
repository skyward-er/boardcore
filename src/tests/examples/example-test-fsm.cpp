/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

// Define STANDALONE_CATCH1_TEST in sbs.conf if you want to run this test alone
// Otherwise, include this file in the sources and compile the
// catch1-tests-entry.cpp entrypoint. This test will be run automatically
// togheter with all the others. Learn more on the skyward-boardcore wiki at:
// https://git.skywarder.eu/r2a/skyward-boardcore/wikis/Testing
#ifdef STANDALONE_CATCH1_TEST
#include "catch1-tests-entry.cpp"
#endif


// We need access to the handleEvent(...) function in state machines in order to
// test them synchronously
#define protected public

#include <catch.hpp>
#include <miosix.h>

#include "example-test-fsm.h"
#include "state_machine_test_helper.h"


TEST_CASE("Testing S1 transitions")
{
    FSMExample fsm;

    SECTION("S1 -> EV_A -> S2")
    {
        REQUIRE(testFSMTransition(fsm, Event{EV_A}, &FSMExample::state_S2));
    }

    SECTION("S1 -> EV_A -> S4 if we go to S3 before")
    {
        // Go to S3 and then back to S1

        REQUIRE(testFSMTransition(fsm, Event{EV_A}, &FSMExample::state_S2));
        REQUIRE(testFSMTransition(fsm, Event{EV_C}, &FSMExample::state_S3));
        REQUIRE(testFSMTransition(fsm, Event{EV_D}, &FSMExample::state_S1));

        // Test if EV_A now puts us in S4
        REQUIRE(testFSMTransition(fsm, Event{EV_A}, &FSMExample::state_S4));
    }
}

TEST_CASE("Testing S2 Transitions")
{
    // These lines are repeated every time we enter a "SECTION". This means that
    // the state machine will be in state S2 at the beginning of every section.
    FSMExample fsm;
    // Move to S2 before testing
    REQUIRE(testFSMTransition(fsm, Event{EV_A}, &FSMExample::state_S2));

    // Test S2 to S3
    SECTION("S2 -> EV_C -> S3")
    {
        REQUIRE(testFSMTransition(fsm, Event{EV_C}, &FSMExample::state_S3));
    }

    // Test S2 to S1
    SECTION("S2 -> EV_B -> S1")
    {
        REQUIRE(testFSMTransition(fsm, Event{EV_B}, &FSMExample::state_S1));
    }
}

TEST_CASE("Testing S3 transitions")
{
    // First move to S3
    FSMExample fsm;
    REQUIRE(testFSMTransition(fsm, Event{EV_A}, &FSMExample::state_S2));
    REQUIRE(testFSMTransition(fsm, Event{EV_C}, &FSMExample::state_S3));

    // Then test S3 transitions

    SECTION("S3 -> EV_D -> S1")
    {
        REQUIRE(testFSMTransition(fsm, Event{EV_D}, &FSMExample::state_S1));
    }

    SECTION("S3 -> EV_A -> S3")
    {
        REQUIRE(testFSMTransition(fsm, Event{EV_A}, &FSMExample::state_S3));
    }
}

TEST_CASE("Testing S4 transitions")
{
    // First move to S4
    FSMExample fsm;
    REQUIRE(testFSMTransition(fsm, Event{EV_A}, &FSMExample::state_S2));
    REQUIRE(testFSMTransition(fsm, Event{EV_C}, &FSMExample::state_S3));
    REQUIRE(testFSMTransition(fsm, Event{EV_D}, &FSMExample::state_S1));
    REQUIRE(testFSMTransition(fsm, Event{EV_A}, &FSMExample::state_S4));

    // Then test S4 transitions

    SECTION("S4 -> EV_D -> S4")
    {
        REQUIRE(testFSMTransition(fsm, Event{EV_D}, &FSMExample::state_S4));
    }

    SECTION("S4 -> EV_A -> S1 (FAIL)")  // This will fail
    {
        REQUIRE(testFSMTransition(fsm, Event{EV_A}, &FSMExample::state_S1));
    }
}