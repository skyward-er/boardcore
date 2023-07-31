/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Author: Luca Erbetta, Davide Mor
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

#include <miosix.h>

#include <type_traits>

#include "CircularBuffer.h"

using miosix::ConditionVariable;
using miosix::FastMutex;
using miosix::Lock;

namespace Boardcore
{

/**
 * Implementation of a synchronized circular buffer
 */
template <typename T, unsigned int Size>
class SyncCircularBuffer
{
public:
    /**
     * Puts a copy of the element in the buffer
     * @param elem element
     */
    void put(const T& elem)
    {
        Lock<FastMutex> l(mutex);
        cv.signal();
        buffer.put(elem);
    }

    /**
     * @brief Gets an element from the buffer, without removing it.
     *
     * Index starts from the oldest element in the buffer.
     * get() returns the first element.
     *
     * @warning Remember to catch the exception!
     * @throw range_error if index >= count().
     * @param i Index of the element to get, starting from the oldest.
     * @return The element.
     */
    T get(unsigned int i = 0)
    {
        Lock<FastMutex> l(mutex);
        return buffer.get(i);
    }

    /**
     * @brief Returns the last element added in the buffer.
     *
     * @warning Remember to catch the exception!
     * @throw range_error if buffer is empty.
     * @return The element.
     */
    T last()
    {
        Lock<FastMutex> l(mutex);
        return buffer.last();
    }

    /**
     * @brief Pops the first element in the buffer.
     *
     * @warning Remember to catch the exception!
     * @throw range_error if buffer is empty.
     * @return The element that has been popped.
     */
    T pop()
    {
        Lock<FastMutex> l(mutex);
        return buffer.pop();
    }

    /**
     * @brief Counts the elements in the buffer.
     *
     * @return Number of elements in the buffer.
     */
    size_t count() const
    {
        Lock<FastMutex> l(mutex);
        return buffer.count();
    }

    bool isEmpty() const
    {
        Lock<FastMutex> l(mutex);
        return buffer.isEmpty();
    }

    bool isFull() const
    {
        Lock<FastMutex> l(mutex);
        return buffer.isFull();
    }

    /**
     * @brief Waits until the buffer contains at least one element.
     */
    void waitUntilNotEmpty()
    {
        Lock<FastMutex> l(mutex);
        while (buffer.isEmpty())
        {
            cv.wait(l);
        }
    }

    /**
     * @brief Returns the maximum number of elements that can be stored in the
     * buffer.
     *
     * @return Buffer size.
     */
    size_t getSize() const { return Size; }

private:
    mutable FastMutex mutex;
    mutable ConditionVariable cv;

    CircularBuffer<T, Size> buffer;
};

}  // namespace Boardcore
