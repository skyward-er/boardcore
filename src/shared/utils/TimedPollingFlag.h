/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <kernel/kernel.h>

#include <functional>

namespace Boardcore
{

/**
 * @brief Pools a flag until it is set or the timeout is reached.
 *
 * @return true if the flag was set, false if the timeout was reached.
 */
inline bool timedPollingFlag(std::function<bool()> readFlag,
                             uint64_t timeout_ns)
{
    // TODO: When Miosix 2.7 will be supported, change this with getTime() in ns
    uint64_t start = miosix::getTick();

    while (miosix::getTick() - start < timeout_ns * 1e6)
    {
        if (readFlag())
        {
            return true;
        }
    }

    return false;
}

}  // namespace Boardcore