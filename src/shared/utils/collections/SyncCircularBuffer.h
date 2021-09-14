/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include "CircularBuffer.h"

using miosix::FastMutex;
using miosix::Lock;

/**
 * Implementation of a synchronized circular buffer
 */
template <typename T, unsigned int Size>
class SyncCircularBuffer : public CircularBuffer<T, Size>
{

    using Super = CircularBuffer<T, Size>;

public:
    /**
     * Puts a copy of the element in the buffer
     * @param elem element
     */
    T& put(const T& elem) override
    {
        Lock<FastMutex> l(mutex);
        return Super::put(elem);
    }

    /**
     * Gets the first element from the buffer, without removing it
     * @warning Remember to catch the exception!
     * @return the element
     * @throws range_error if buffer is empty
     */
    T& get() override
    {
        Lock<FastMutex> l(mutex);
        return Super::get();
    }

    /**
     * Gets an element from the buffer, without removing it
     * Index starts at the element returned by get() or pop(): get(0) is
     * the same as get()
     *
     * @warning Remember to catch the exception!
     * @return the element
     * @throws range_error if buffer is empty
     */
    T& get(unsigned int i) override
    {
        Lock<FastMutex> l(mutex);
        return Super::get(i);
    }

    /**
     * Pops the first element in the buffer.
     * @warning Remember to catch the exception!
     * @return the element that has been popped
     * @throws range_error if buffer is empty
     */
    const T& pop() override
    {
        Lock<FastMutex> l(mutex);
        return Super::pop();
    }

    /**
     * Counts the elements in the buffer
     * @return number of elements in the buffer
     */
    size_t count() const override
    {
        Lock<FastMutex> l(mutex);
        return Super::count();
    }

    bool isEmpty() const override
    {
        Lock<FastMutex> l(mutex);
        return Super::isEmpty();
    }

    bool isFull() const override
    {
        Lock<FastMutex> l(mutex);
        return Super::isFull();
    }

private:
    mutable FastMutex mutex;
};
