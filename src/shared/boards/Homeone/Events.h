/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
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
#ifndef SRC_SHARED_BOARDS_HOMEONE_EVENTS_H
#define SRC_SHARED_BOARDS_HOMEONE_EVENTS_H

#include "events/Event.h"

namespace HomeoneBoard
{
/**
 * Definition of all events in the Homeone Board software
 */
enum Events : uint8_t
{
    EV_ARM = EV_FIRST_SIGNAL,
    EV_DISARM,

    EV_ENABLE_TEST_MODE,
    EV_RESET_BOARD,

    EV_START_LAUNCH,
    EV_ABORT_LAUNCH,

    EV_UNBILICAL_DETATCHED,

    EV_FMM_ASCENT_TIMEOUT,

    EV_ADA_START,
    EV_ADA_GO_ACTIVE,
    EV_ADA_STOP,

    EV_APOGEE_DETECTED,
    EV_DEPLOY_MAIN_PARACHUTE,

    EV_STOP_SAMPLING
};
}

#endif /* SRC_SHARED_BOARDS_HOMEONE_EVENTS_H */
