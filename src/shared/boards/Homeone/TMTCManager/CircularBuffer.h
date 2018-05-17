/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron
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

#ifndef CIRCULARBUFFER_H
#define CIRCULARBUFFER_H

#include <miosix.h>
#include <stdio.h>

class CircularBuffer
{
public:
    /**
     * Create a buffer with given dimension (uses dynamic allocation).
     */
    CircularBuffer(const uint32_t totalSize)
    {
        this->totalSize = totalSize;
        buffer          = new uint8_t[totalSize];

        memset(buffer, 0, totalSize);
    }

    /**
     * Calculate the occupied portion of the buffer.
     */

    uint32_t occupiedSize()
    {
        miosix::Lock<miosix::FastMutex> l(mutex);
        if (full)
            return totalSize;
        else if (last < first)
            return totalSize - (first - last);
        else
            return last - first;
    }

    /**
     * Calculate the free portion of the buffer.
     */
    uint32_t freeSize() { return totalSize - occupiedSize(); }

    /**
     * Read (removing from the queue) at maximum n chars and
     * stores them in *buf in subsequent positions.
     *
     * \param buf   where to store the read chars
     * \param len   how many characters to read (at maximum)
     * \return      number of read characters (could be less than len
     *              if the buffer contains less than n characters)
     */
    uint32_t read(uint8_t* buf, uint32_t len)
    {
        uint32_t occupied_size = occupiedSize();
        uint32_t availableLen  = occupied_size > len ? len : occupied_size;
        uint32_t i             = 0;

        {
            miosix::Lock<miosix::FastMutex> l(mutex);

            for (i = 0; i < availableLen; i++)
            {

                if (i == 0 && full)
                    full = false;

                buf[i] = buffer[first];

                first++;
                first %= totalSize;

                if (first == last)
                    break;
            }
        }

        return i;
    }

    /**
     * Write n characters in the buffer (or until it is full).
     *
     * \param buf   pointer to the message to write
     * \param len   how many characters to write (at maximum)
     * \return      number of chars written
     */
    uint32_t write(const uint8_t* buf, uint32_t len)
    {
        uint32_t i = 0;

        {
            miosix::Lock<miosix::FastMutex> l(mutex);

            if (!full)
            {
                for (i = 0; i < len; i++)
                {

                    buffer[last] = buf[i];

                    last++;
                    last %= totalSize;

                    if (last == first)
                    {
                        full = true;
                        break;
                    }
                }
            }
        }

        return i;
    }

    /*
     * Print the whole content of the buffer (for debug purposes).
     */
    void printContent()
    {
        printf("Buffer: first %lu last %lu occupied %lu total size %lu\n",
               first, last, occupiedSize(), totalSize);

        uint32_t offset = first;
        for (uint8_t i = 0; i < occupiedSize(); i++)
        {
            printf("%c(%d) ", buffer[offset], buffer[offset]);

            offset++;
            offset %= totalSize;
        }
        printf("\n");
    }

private:
    uint32_t totalSize;
    uint32_t first = 0;
    uint32_t last  = 0;
    bool full      = false;
    uint8_t* buffer;

    miosix::FastMutex mutex;
};

#endif /* CIRCULARBUFFER_H */
