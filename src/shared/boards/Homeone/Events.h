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

// Common includes
#include "Topics.h"
#include "events/EventBroker.h"

namespace HomeoneBoard
{
/**
 * Definition of all events in the Homeone Board software
 *
 *
 * Event naming convention:
 *
 * EV_<COMPONENT>_<EVENT-NAME>
 *
 * Example:
 *
 * Component: Flight Mode Manager (FMM)
 * Event name: Apogee
 * --> EV_FMM_APOGEE
 *
 */
enum Events : uint8_t
{
    // Flight Mode manager
    EV_FMM_ARM = EV_FIRST_SIGNAL,
    EV_FMM_DISARM,
    EV_FMM_LAUNCH,
    EV_FMM_ASCENT_TIMEOUT,
    EV_FMM_APOGEE,
    EV_FMM_MAIN_PARACHUTE_DEPLOY,

    // Sensor manager
    EV_SM_START_SAMPLING
};
}

#endif /* SRC_SHARED_BOARDS_HOMEONE_EVENTS_H */
