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
 * Refer to section 5.1.1 of the Software Design Document.
 */
enum Events : uint8_t
{
    EV_PING_RECEIVED           = 1,
    EV_HEARTBEAT_RECEIVED      = 2,
    EV_SHUTDOWN_COMPLETE       = 3,
    EV_NOSECONE_STATUS_REQUEST = 4,
    EV_IGNITION_STATUS_REQUEST = 5,
    EV_NOSECONE_STATUS         = 6,
    EV_IGNITION_STATUS         = 7,

    EV_ALTIMETER_CALIBRATION = 32,
    EV_START_SAMPLING        = 40,
    EV_STOP_SAMPLING         = 41,
    EV_NOSECONE_OPEN         = 42,
    EV_NOSECONE_CLOSE        = 43,
    EV_TEST_MODE             = 48,
    EV_RESET_BOARD           = 49,

    EV_ARM                    = 64,
    EV_DISARM                 = 65,
    EV_START_LAUNCH           = 72,
    EV_ABORT_LAUNCH           = 73,
    EV_UMBILICAL_DISCONNECTED = 80,
    EV_ASCENT_TIMEOUT         = 81,
    EV_APOGEE_DETECTED        = 82,
    EV_APOGEE_TIMEOUT         = 83,
    EV_MAIN_CHUTE_ALTITUDE    = 84,
    EV_DESCENT_TIMEOUT        = 85,

    EV_TC_START_LAUNCH          = 128,
    EV_TC_START_SAMPLING        = 136,
    EV_TC_STOP_SAMPLING         = 137,
    EV_TC_NOSECONE_OPEN         = 144,
    EV_TC_NOSECONE_CLOSE        = 145,
    EV_TC_TEST_MODE             = 152,
    EV_TC_RESET_BOARD           = 160,
    EV_TC_ALTIMETER_CALIBRATION = 168,

    EV_PRESSURE_SAMPLE = 192

};

struct AltimeterCalibrationEvent : Event
{
    uint16_t T0;  // Calibration temperature
    uint16_t P0;  // Calibration pressure
};
}

#endif /* SRC_SHARED_BOARDS_HOMEONE_EVENTS_H */
