/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron
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
#pragma once
#include <Common.h>

#include <events/EventBroker.h>
#include <boards/Nosecone/Events.h>
#include <boards/Nosecone/Topics.h>

#define UNUSED(x) (void)(x)

static const unsigned int  r_end_port = GPIOG_BASE;
static const unsigned char r_end_pin = 6;
static const unsigned int  l_end_port = GPIOG_BASE;
static const unsigned char l_end_pin = 7;
typedef miosix::Gpio<r_end_port, r_end_pin> RightMotorEnd;
typedef miosix::Gpio<l_end_port, l_end_pin> LeftMotorEnd;

namespace NoseconeBoard
{
namespace MotorLimit
{

/**
 * These handlers are used by the pinObserver to post an event
 * whenever the motor limit is reached (finecors) .
 */
static void r_finecorsaHandler(unsigned int p, unsigned char c) {
    UNUSED(p);
    UNUSED(c);
    sEventBroker->post(Event{EV_MOTOR_LIMIT}, TOPIC_NOSECONE);
}

static void l_finecorsaHandler(unsigned int p, unsigned char c) {
    UNUSED(p);
    UNUSED(c);
    sEventBroker->post(Event{EV_MOTOR_LIMIT}, TOPIC_NOSECONE);
}

/**
 * @brief Start observing limit pins.
 */
static void observeLimitPins(PinObserver* pinObs)
{
    /* Set motor limit pins */
    RightMotorEnd::mode(miosix::Mode::INPUT);
    LeftMotorEnd::mode(miosix::Mode::INPUT);

    /* Assign handlers for the motor limit pins */
    pinObs->observePin(r_end_port, r_end_pin, PinObserver::Trigger::FALLING_EDGE, 
                            r_finecorsaHandler);
    pinObs->observePin(l_end_port, l_end_pin, PinObserver::Trigger::FALLING_EDGE, 
                            l_finecorsaHandler);
}

} /* namespace MotorLimit */
} /* namespace NoseconeBoard */