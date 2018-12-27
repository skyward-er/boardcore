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

namespace NoseconeBoard
{
namespace MotorLimit
{

static const unsigned int  r_limit_port = GPIOG_BASE;
static const unsigned char r_limit_pin = 6;
static const unsigned int  l_limit_port = GPIOG_BASE;
static const unsigned char l_limit_pin = 7;

typedef miosix::Gpio<r_limit_port, r_limit_pin> RightMotorLimit;
typedef miosix::Gpio<l_limit_port, l_limit_pin> LeftMotorLimit;

/**
 * These handlers are used by the pinObserver to post an event
 * whenever the motor limit is reached (finecors) .
 */
static void r_finecorsaHandler(unsigned int p, unsigned char c) {
    UNUSED(p);
    UNUSED(c);
    sEventBroker->post(Event{EV_R_MOTOR_LIMIT}, TOPIC_NOSECONE);
}

static void l_finecorsaHandler(unsigned int p, unsigned char c) {
    UNUSED(p);
    UNUSED(c);
    sEventBroker->post(Event{EV_L_MOTOR_LIMIT}, TOPIC_NOSECONE);
}

/**
 * @brief Start observing limit pins.
 */
static void observeLimitPins(PinObserver* pinObs)
{
    /* Set motor limit pins */
    RightMotorLimit::mode(miosix::Mode::INPUT);
    LeftMotorLimit::mode(miosix::Mode::INPUT);

    /* Assign handlers for the motor limit pins */
    pinObs->observePin(r_limit_port, r_limit_pin, PinObserver::Trigger::FALLING_EDGE, 
                            r_finecorsaHandler);
    pinObs->observePin(l_limit_port, l_limit_pin, PinObserver::Trigger::FALLING_EDGE, 
                            l_finecorsaHandler);
}

} /* namespace MotorLimit */
} /* namespace NoseconeBoard */