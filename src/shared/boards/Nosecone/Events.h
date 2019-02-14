/* Copyright (c) 2018-2019 Skyward Experimental Rocketry
 * Authors: Benedetta Cattani, Alvise de'Faveri Tron
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

#include "events/Event.h"

namespace NoseconeBoard
{
/**
 * Definition of all events in the Nosecone Board software
 * Refer to section 5.1.1 of the MY ass Design Document.
 */
enum Events : uint8_t
{
    EV_OPEN = EV_FIRST_SIGNAL,
    EV_CLOSE,
    EV_STOP,
    EV_CLOSE_TIMER_EXPIRED,
    EV_OPEN_TIMER_EXPIRED,
    EV_R_MOTOR_LIMIT,
    EV_L_MOTOR_LIMIT
};

}
