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

#include <utils/Span.h>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

/**
 * @brief A ring buffer allocator. Allocates variable-size chunks in a circular
 * way.
 *
 * As opposed to @ref CircularBuffer, which is typed for a particular type T,
 * and can only insert or remove elements of that type, this buffer can store
 * arbitrary data of variable size. Its main use is to implement a
 * multiple-producers single-consumer queue of variable-sized data.
 *
 * @note Chunks are always stored in a contiguous manner, at the expense of some
 * wasted space at the end of the buffer, depending on the allocation state of
 * the buffer.
 * @details Each chunk is made of a known-size header (SizeType), which stores
 * the size of the following data element, and the data itself.
 *
 * @tparam Size The size of the buffer in bytes. Must be greater than
 * sizeof(Header).
 * @tparam SizeType The type of the size field used to store the size of each
 * data chunk. Defaults to uint32_t for performance reasons (alignment), but can
 * be changed to a smaller type to reduce the header overhead.
 */
template <size_t Size, typename SizeType>
class CircularAllocator
{
public:
    std::span<uint8_t> tryGet()
    {
        if (isEmpty())
            return {};

        if (readPos + sizeof(SizeType) > activeSize)
            readPos = 0;  // Wrap around if we are at the end of the buffer

        if (isEmpty())
            return {};

        SizeType size;
        std::memcpy(&size, buffer + readPos, sizeof(SizeType));
        uint8_t* dataPtr = buffer + readPos + sizeof(SizeType);

        std::printf("tryGet: returning chunk at %d, size: %d\n", readPos, size);
        return {dataPtr, size};
    }

    void pop()
    {
        std::printf("pop: is there anything to pop?\n");

        if (isEmpty())
            return;

        if (readPos + sizeof(SizeType) > activeSize)
            readPos = 0;  // Wrap around if we are at the end of the buffer

        if (isEmpty())
            return;

        SizeType size;
        std::memcpy(&size, buffer + readPos, sizeof(SizeType));
        std::printf("pop: popping chunk at %d, size %d\n", readPos, size);
        readPos += sizeof(SizeType) + size;
    }

    bool tryPut(std::span<const uint8_t> dataSpan)
    {
        assert(dataSpan.size() <= std::numeric_limits<SizeType>::max());

        const uint8_t* data = dataSpan.data();
        SizeType size       = dataSpan.size();
        size_t chunkSize    = sizeof(SizeType) + size;
        size_t newPos       = writePos + chunkSize;

        std::printf(
            "tryPut: chunkSize: %d, readPos: %d, writePos: %d, activeSize: "
            "%d\n",
            chunkSize, readPos, writePos, activeSize);

        if (writePos < readPos && newPos >= readPos)
            return false;

        // Is there enough space at the end of the buffer?
        if (newPos <= Size)
        {
            // Store the header and the data
            std::memcpy(buffer + writePos, &size, sizeof(SizeType));
            std::memcpy(buffer + writePos + sizeof(SizeType), data, size);

            // Grow the active size if this chunk went past it
            if (newPos > activeSize)
                activeSize = newPos;

            writePos = newPos;

            std::printf(
                "tryPut: inserted at end of buf, readPos: %d, writePos: %d, "
                "activeSize: %d\n",
                readPos, writePos, activeSize);
            return true;
        }

        // If we cannot fit the new chunk at the end of the buffer, we try to
        // allocate it at the beginning so that we can keep the data contiguous
        newPos = chunkSize;
        if (newPos >= readPos)
            return false;

        // Store the header and the data
        std::memcpy(buffer, &size, sizeof(SizeType));
        std::memcpy(buffer + sizeof(SizeType), data, size);
        // Shrink the active size to account for the ignored space at the end
        activeSize = writePos;
        writePos   = newPos;

        std::printf(
            "tryPut: inserted at start of buf, readPos: %d, writePos: %d, "
            "activeSize: %d\n",
            readPos, writePos, activeSize);
        return true;
    }

    bool isEmpty() { return readPos == writePos; }

    size_t capacity() { return Size; }

private:
    uint8_t buffer[Size];
    size_t readPos    = 0;
    size_t writePos   = 0;
    size_t activeSize = 0;
};
