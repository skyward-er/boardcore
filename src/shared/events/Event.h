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

#pragma once

#include <stdint.h>

namespace Boardcore
{

enum EventSignal : uint8_t
{
    EV_ENTRY        = 0,
    EV_EXIT         = 1,
    EV_EMPTY        = 2,
    EV_INIT         = 3,
    EV_FIRST_SIGNAL = 4
};

/**
 * 	Example definiton of custom signals:

        enum CustomSignal : uint8_t
        {
                SIG_ONE = SG_FIRST_SIGNAL,
                SIG_TWO,
                SIG_THREE,
                SIG_FOUR
        };

 */

struct Event
{
    uint8_t code;
};

/* Example of extended Event structure

        struct ExtendedEvent : public Event{
                uint32_t customMember;
        };
*/

}  // namespace Boardcore
