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

#pragma once

#include <cstring>
#include <stdexcept>
#include <type_traits>

using std::range_error;

/**
 * Implementation of an non-synchronized circular buffer
 */
template <typename T, unsigned int Size>
class CircularBuffer
{
    static_assert(Size > 0, "Circular buffer size must be greater than 0!");

public:
    CircularBuffer() {}
    virtual ~CircularBuffer() {}

    /**
     * Puts a copy of the element in the buffer
     * @param elem element
     */
    virtual T& put(const T& elem)
    {
        buffer[write_ptr] = elem;
        T& added          = buffer[write_ptr];

        if (!empty && write_ptr == read_ptr)
        {
            read_ptr = (read_ptr + 1) % Size;
        }
        write_ptr = (write_ptr + 1) % Size;

        empty = false;

        return added;
    }

    /**
     * Gets an element from the buffer, without removing it
     * Index starts from the oldest element in the buffer: get(0) returns the
     * same element as get()
     *
     * @warning Remember to catch the exception!
     * @throw range_error if index >= count()
     *
     * @param i Index of the elemnt to get, starting from the oldest
     *
     * @return the element
     */
    virtual T& get(unsigned int i)
    {
        if (i < CircularBuffer<T, Size>::count())
        {
            int ptr = (read_ptr + i) % Size;
            return buffer[ptr];
        }
        else
            throw range_error("Index out of range");
    }

    /**
     * @brief Returns the last element added in the buffer.
     *
     * @throw range_error if buffer is empty
     * @warning Remember to catch the exception!
     * @return the element
     */
    virtual T& last() { return get(count() - 1); }

    /**
     * Gets the first element from the buffer, without removing it
     * @throw range_error if buffer is empty
     * @warning Remember to catch the exception!
     * @return the element
     */
    virtual T& get()
    {
        if (!empty)
        {
            return buffer[read_ptr];
        }
        else
            throw range_error("CircularBuffer is empty!");
    }

    /**
     * Pops the first element in the buffer.
     * @throw range_error if buffer is empty
     * @warning Remember to catch the exception!
     * @return the element that has been popped
     */
    virtual const T& pop()
    {
        if (!empty)
        {
            size_t ptr = read_ptr;
            read_ptr   = (read_ptr + 1) % Size;

            empty = read_ptr == write_ptr;

            return buffer[ptr];
        }
        else
            throw range_error("CircularBuffer is empty!");
    }

    /**
     * Counts the elements in the buffer
     * @return number of elements in the buffer
     */
    virtual size_t count() const
    {
        if (empty)
            return 0;
        if (write_ptr > read_ptr)
        {
            return write_ptr - read_ptr;
        }
        else
        {
            return Size - read_ptr + write_ptr;
        }
    }

    virtual bool isEmpty() const { return empty; }

    virtual bool isFull() const
    {
        return CircularBuffer<T, Size>::count() == Size;
    }
    /**
     * Returns the maximum number of elements that can be stored in the buffer
     * @return buffer size
     */
    size_t getSize() const { return Size; }

protected:
    T buffer[Size];

    size_t write_ptr = 0, read_ptr = 0;
    bool empty = true;
};
