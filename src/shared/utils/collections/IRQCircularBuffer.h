/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Davide Mor
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

using miosix::FastInterruptDisableLock;
using miosix::FastInterruptEnableLock;
using miosix::Thread;

namespace Boardcore
{

/**
 * Implementation of a synchronized circular buffer that can be used inside
 * interrupt service routines.
 */
template <typename T, unsigned int Size>
class IRQCircularBuffer
{
    static_assert(std::is_trivially_copy_constructible<T>::value,
                  "T must be trivially copy constructible!");

public:
    /**
     * @brief Puts a copy of the element in the buffer.
     */
    T& put(const T& elem)
    {
        FastInterruptDisableLock d;
        IRQwakeWaitingThread();
        return buffer.put(elem);
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
        FastInterruptDisableLock d;
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
        FastInterruptDisableLock d;
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
        FastInterruptDisableLock d;
        return buffer.pop();
    }

    /**
     * @brief Counts the elements in the buffer.
     *
     * @return Number of elements in the buffer.
     */
    size_t count() const
    {
        FastInterruptDisableLock d;
        return buffer.count();
    }

    bool isEmpty() const
    {
        FastInterruptDisableLock d;
        return buffer.isEmpty();
    }

    bool isFull() const
    {
        FastInterruptDisableLock d;
        return buffer.isFull();
    }

    /**
     * @brief Puts a copy of the element in the buffer.
     *
     * @warning Only to be called inside an ISR or with interrupts disabled.
     */
    void IRQput(const T& elem)
    {
        IRQwakeWaitingThread();
        buffer.put(elem);
    }

    /**
     * @brief Puts a copy of the element in the buffer.
     *
     * @warning Only to be called inside an ISR or with interrupts disabled.
     *
     * @param elem element
     * @param hppw Set to true if the woken thread is higher priority than the
     * current one, unchanged otherwise
     */
    void IRQput(const T& elem, bool& hppw)
    {
        if (waiting && (waiting->IRQgetPriority() >
                        Thread::IRQgetCurrentThread()->IRQgetPriority()))
            hppw = true;

        IRQwakeWaitingThread();
        buffer.put(elem);
    }

    /**
     * @brief Gets an element from the buffer, without removing it.
     *
     * @warning Only to be called inside an ISR or with interrupts disabled.
     *
     * Index starts from the oldest element in the buffer.
     * get() returns the first element.
     *
     * @warning Remember to catch the exception!
     * @throw range_error if index >= count().
     * @param i Index of the element to get, starting from the oldest.
     * @return The element.
     */
    T IRQget(unsigned int i = 0) { return buffer.get(i); }

    /**
     * @brief Pops the first element in the buffer.
     *
     * @warning Only to be called inside an ISR or with interrupts disabled.
     *
     * @warning Remember to catch the exception!
     * @throw range_error if buffer is empty.
     * @return The element that has been popped.
     */
    T IRQpop() { return buffer.pop(); }

    /**
     * @brief Counts the elements in the buffer.
     *
     * @warning Only to be called inside an ISR or with interrupts disabled.
     *
     * @return Number of elements in the buffer.
     */
    size_t IRQcount() const { return buffer.count(); }

    /**
     * @warning Only to be called inside an ISR or with interrupts disabled.
     */
    bool IRQisEmpty() const { return buffer.isEmpty(); }

    /**
     * @warning Only to be called inside an ISR or with interrupts disabled.
     */
    bool IRQisFull() const { return buffer.isFull(); }

    /**
     * @brief Waits until the buffer contains at least one element.
     */
    void waitUntilNotEmpty()
    {
        FastInterruptDisableLock d;
        while (buffer.isEmpty())
        {
            IRQwakeWaitingThread();
            waiting = Thread::IRQgetCurrentThread();

            Thread::IRQwait();
            {
                FastInterruptEnableLock e(d);
                Thread::yield();
            }
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
    void IRQwakeWaitingThread()
    {
        if (waiting)
        {
            waiting->IRQwakeup();
            waiting = 0;
        }
    }

    Thread* waiting = nullptr;
    CircularBuffer<T, Size> buffer;
};

}  // namespace Boardcore
