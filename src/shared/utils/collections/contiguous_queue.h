/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Author: Federico Terraneo
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

#include <stdexcept>

namespace Boardcore
{

/**
 * A fixed size FIFO queue whose elements are stored in a contiguous array,
 * which allows direct access to the array in order to add/remove multiple
 * elements.
 *
 * NOTE: This queue is most efficent if the data is removed form the queue in
 * large chunks. Removing one element at a time is most inefficient since
 * shifting all the other elements in the queue is needed. There are no
 * performance constraints on inserting.
 *
 * The queue is not synchronized.
 *
 * \param T type of the element
 * \param N maximum number of elements (fixed size queue)
 */
template <typename T, unsigned N>
class ContiguousQueue
{
public:
    /**
     * Constructor
     */
    ContiguousQueue() {}

    /**
     * \return a pointer to the side of the queue where elements can be added.
     * Call availableToAdd() to know how much space is left. The pointer is
     * meant to be used as an array, so as to add more than one item.
     *
     * NOTE: if availableToAdd() returns 0 the pointer is past the end of the
     * array and should not be dereferenced
     */
    T* addEnd() { return elements + size; }

    /**
     * \return a pointer to the side of the queue where elements can be removed.
     * Call availableToRemove() to know how many elements are available. The
     * pointer is meant to be used as an array, so as to read more than one
     * item.
     *
     * NOTE: if availableToRemove() returns 0 the pointer should not be
     * dereferenced
     */
    T* removeEnd() { return elements; }

    /**
     * \return a pointer to the side of the queue where elements can be removed.
     * Call availableToRemove() to know how many elements are available. The
     * pointer is meant to be used as an array, so as to read more than one
     * item.
     *
     * NOTE: if availableToRemove() returns 0 the pointer should not be
     * dereferenced
     */
    const T* removeEnd() const { return elements; }

    /**
     * \return how much space is available to add elements
     */
    unsigned int availableToAdd() const { return N - size; }

    /**
     * \return how many items are available to read
     */
    unsigned int availableToRemove() const { return size; }

    /**
     * After having added elements to the pointer obtained through addEnd(),
     * call this function to let the queue know how many elements have been
     * added
     * \param n elements added
     * \throws range_error if too many elements were addded. Memory corruption
     * has already occurred at this point, though
     */
    void added(unsigned int n)
    {
        if (size + n > N)
            throw std::range_error("ContiguousBuffer::added");
        size += n;
    }

    /**
     * After having removed elements to the pointer obtained through
     * removeEnd(), call this function to let the queue know how many elements
     * have been added removed
     * \param n elements removed
     * \throws range_error if too many elements were removed. Memory corruption
     * has already occurred at this point, though
     */
    void removed(unsigned int n)
    {
        if (n > size)
            throw std::range_error("ContiguousBuffer::removed");
        if (n == 0)
            return;
        for (unsigned int i = 0; i < size - n; i++)
            elements[i] = std::move(elements[i + n]);
        size -= n;
    }

private:
    ContiguousQueue(const ContiguousQueue&)            = delete;
    ContiguousQueue& operator=(const ContiguousQueue&) = delete;

    T elements[N]     = {0};
    unsigned int size = 0;
};

}  // namespace Boardcore
