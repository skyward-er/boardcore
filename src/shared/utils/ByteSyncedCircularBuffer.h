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

#ifndef BYTESYNCEDCIRCULARBUFFER_H
#define BYTESYNCEDCIRCULARBUFFER_H

#include <miosix.h>
#include <iostream>
#include <cstdint>

using namespace std;

/**
 * FIFO synced fixed-length buffer of bytes.
 **/
class ByteSyncedCircularBuffer
{
public:
    /**
     * @param byteSize   size of the buffer in bytes
     * @param overwrite  whether or not the last element should override the first
    *                    if there isn't enough space
     */
    ByteSyncedCircularBuffer(const size_t byteSize, const bool overwrite = false) 
        : totalSize(byteSize), buffer(new uint8_t[byteSize]), overwrite(overwrite) {

        }

    ~ByteSyncedCircularBuffer() { delete[] buffer; }

     /**
     * Write n characters in the buffer (if there's enough space).
     *
     * @param buf   pointer to the message to write
     * @param len   how many characters to write
     * @return      number of chars written
     */
    size_t put(const uint8_t* buf, const size_t len)
    {
        /* Check that there's enough space to store the element */
        if(!overwrite && (totalSize - count(false)) < len)
            return 0;

        miosix::Lock<miosix::FastMutex> l(mutex);

        /* Write byte per byte */
        size_t i = 0;
        while (i < len)
        {
            buffer[write_ptr] = buf[i];

            i++;
            write_ptr = (write_ptr + 1) % totalSize;

            /* Move read pointer if overwriting */
            if(overwrite && full){
                read_ptr = write_ptr;
            }
            else if (write_ptr == read_ptr)
            {
                full = true;
                break;
            }
        }

        return i;
    }

    /**
     * FIFO read at maximum n chars from the buffer and
     * store them in buf in subsequent positions.
     *
     * @param buf   where to store the read chars
     * @param len   how many characters to read (at maximum)
     * @return      number of read characters (could be less than len
     *              if the buffer contains less than n characters)
     */
    size_t pop(uint8_t* buf, const size_t len)
    {
        /* Check that the buffer is not empty */
        if(count(false) == 0)
            return 0;

        miosix::Lock<miosix::FastMutex> l(mutex);

        size_t i = 0;
        while (i < len)
        {
            if (full) full = false;

            buf[i] = buffer[read_ptr];

            i++;
            read_ptr = (read_ptr + 1) % totalSize;

            if (read_ptr == write_ptr)
                break;
        }

        return i;
    }


    /**
     * Count the number of occupied bytes in the buffer.
     * @return number of readable bytes in the buffer
     */

    size_t count(bool synced=true)
    {
        if(synced)
            miosix::Lock<miosix::FastMutex> l(mutex);

        if (full)
            return totalSize;
        else if (write_ptr < read_ptr)
            return totalSize - (read_ptr - write_ptr);
        else
            return write_ptr - read_ptr;
    }

    bool isEmpty() { return (write_ptr==read_ptr && !full); }

    /**
     * Returns the maximum number of bytes that can be stored in the buffer
     * @return buffer size
     */
    size_t getSize() { return totalSize; }

    friend ostream& operator << (ostream& os, const ByteSyncedCircularBuffer& buffer)
    {
        os << "Size: "<< buffer.totalSize << "  Read ptr:  " << buffer.read_ptr
          << "  Write ptr:  "<< buffer.write_ptr << endl << "Content:  " << buffer.buffer << endl;

       return os;
    }

private:
    const size_t totalSize;
    uint8_t* buffer;
    size_t read_ptr = 0, write_ptr  = 0;

    bool overwrite = false;
    bool full      = false;

    miosix::FastMutex mutex;
};

#endif /* BYTESYNCEDCIRCULARBUFFER_H */
