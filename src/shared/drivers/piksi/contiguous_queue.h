/***************************************************************************
 *   Copyright (C) 2017 by Terraneo Federico                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

#ifndef CONTIGUOUS_QUEUE
#define CONTIGUOUS_QUEUE

#include <stdexcept>

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
    ContiguousQueue(const ContiguousQueue&) = delete;
    ContiguousQueue& operator=(const ContiguousQueue&) = delete;

    T elements[N] = {0};
    unsigned int size = 0;
};

#endif  // CONTIGUOUS_QUEUE
