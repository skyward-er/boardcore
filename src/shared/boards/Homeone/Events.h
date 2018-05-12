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
    EV_PING_RECEIVED           = 1,
    EV_SHUTDOWN_COMPLETE       = 2,
    EV_NOSECONE_STATUS_REQUEST = 3,
    EV_IGNITION_STATUS_REQUEST = 4,
    EV_ARM                     = 32,
    EV_DISARM                  = 33,
    EV_ABORT_LAUNCH            = 34,
    EV_UMBILICAL_DISCONNECTED  = 35,
    EV_APOGEE_DETECTED         = 36,
    EV_APOGEE_TIMEOUT          = 37,
    EV_MAIN_CHUTE_ALTITUDE     = 38,
    EV_DESCENT_TIMEOUT         = 39,
    EV_START_SAMPLING          = 64,
    EV_STOP_SAMPLING           = 65,
    EV_START_LAUNCH            = 66,
    EV_NOSECONE_CLOSE          = 67,
    EV_NOSECONE_OPEN           = 68,
    EV_TEST_MODE               = 69,
    EV_RESET_BOARD             = 70,
    EV_BAROMETER_CALIBRATION   = 128,
    EV_PRESSURE_SAMPLE         = 192
};
}

#endif /* SRC_SHARED_BOARDS_HOMEONE_EVENTS_H */
