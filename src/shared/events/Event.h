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

typedef uint8_t Event;

enum BasicEvent : Event
{
    EV_ENTRY        = 0,
    EV_EXIT         = 1,
    EV_EMPTY        = 2,
    EV_INIT         = 3,
    EV_FIRST_CUSTOM = 4
};

/**
 * Example definition of custom events:
 *
 * enum CustomEvent : Event
 * {
 *     EV_ONE = EV_FIRST_CUSTOM,
 *     EV_TWO,
 *     EV_THREE.
 *     EV_FOUR
 * }
 */

}  // namespace Boardcore
