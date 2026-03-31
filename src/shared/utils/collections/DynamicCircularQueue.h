/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <kernel/sync.h>
#include <utils/collections/CircularAllocator.h>

// template <size_t Size = 256, typename SizeType = uint32_t>
class DynamicCircularQueue : private CircularAllocator<254, uint32_t>
{
    using Super = CircularAllocator<254, uint32_t>;

public:
    std::span<const uint8_t> tryGet()
    {
        miosix::Lock<miosix::FastMutex> lock(mutex);
        return Super::tryGet();
    }

    bool tryPut(std::span<const uint8_t> dataSpan)
    {
        miosix::Lock<miosix::FastMutex> lock(mutex);
        return Super::tryPut(dataSpan);
    }

    template <typename T>
    const T& get()
    {
    }

    template <typename T>
    void put(const T& elem)
    {
        popCv.wait()
    }

private:
    miosix::FastMutex mutex;
    miosix::ConditionVariable putCv;  // Notifies on element put in queue
    miosix::ConditionVariable popCv;  // Notifies on element popped from queue
};
