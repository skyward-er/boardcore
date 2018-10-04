/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
#ifndef SRC_SHARED_UTILS_SYNCEDCIRCULARBUFFER_H
#define SRC_SHARED_UTILS_SYNCEDCIRCULARBUFFER_H

#include <miosix.h>
#include <cstring>
#include <stdexcept>

using std::runtime_error;
using miosix::FastMutex;
using miosix::Lock;

/**
 * Implementation of an synchronized circular buffer
 */
template <typename T>
class SyncedCircularBuffer
{
public:
    SyncedCircularBuffer(size_t size) : size(size), buffer(new T[size]) {}
    ~SyncedCircularBuffer() { delete[] buffer; }

    /**
     * Puts a copy of the element in the buffer
     * @param elem element
     */
    SyncedCircularBuffer<T>& put(const T& elem)
    {
        Lock<FastMutex> l(mutex);
        buffer[write_ptr] = elem;

        if (!empty && write_ptr == read_ptr)
        {
            read_ptr = (read_ptr + 1) % size;
        }
        write_ptr = (write_ptr + 1) % size;

        empty = false;
        return *this;
    }

    /**
     * Gets the first element from the buffer, without removing it
     * @warning Always catch the exception, because the buffer may have been
     * emptied after the last call to isEmpty.
     * @return the element
     * @throws runtime_error if buffer is empty
     */
    const T& get()
    {
        Lock<FastMutex> l(mutex);
        if (!empty)
        {
            return buffer[read_ptr];
        }
        else
            throw runtime_error("CircularBuffer is empty!");
    }

    /**
     * Pops the first element in the buffer.
     * @warning Always catch the exception, because the buffer may have been
     * emptied after the last call to isEmpty.
     * @return the element that has been popped
     * @throws runtime_error if buffer is empty
     */
    const T& pop()
    {
        Lock<FastMutex> l(mutex);
        if (!empty)
        {
            size_t ptr = read_ptr;
            read_ptr   = (read_ptr + 1) % size;

            if (read_ptr == write_ptr)
                empty = true;

            return buffer[ptr];
        }
        else
            throw runtime_error("CircularBuffer is empty!");
    }

    /**
     * Counts the elements in the buffer
     * @return number of elements in the buffer
     */
    size_t count()
    {
        Lock<FastMutex> l(mutex);
        if (empty)
            return 0;
        if (write_ptr > read_ptr)
        {
            return write_ptr - read_ptr;
        }
        else
        {
            return size - read_ptr + write_ptr;
        }
    }

    bool isEmpty() { return empty; }

    /**
     * Returns the maximum number of elements that can be stored in the buffer
     * @return buffer size
     */
    size_t getSize() { return size; }

private:
    const size_t size;
    T* buffer;
    size_t write_ptr = 0, read_ptr = 0;
    bool empty = true;
    FastMutex mutex;
};

#endif /* SRC_SHARED_UTILS_SYNCEDCIRCULARBUFFER_H */
