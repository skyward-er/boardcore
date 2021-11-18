/* Copyright (c) 2016-2017 Skyward Experimental Rocketry
 * Author: Silvano Seva
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

#include <Common.h>
#include "packet.h"

/**
 * Quick and dirty implementation of a ring buffer. All the operations are
 * guaranteed to be atomic through the use of a mutex
 */
class PacketBuffer
{
public:
    /**
     *  @param storageSize: size of internal buffer in bytes
     *  internal buffer is made of uint8_t
     */
    explicit PacketBuffer(size_t storageSize);

    ~PacketBuffer();

    /**
     * Check if internal buffer's allocation succeeded. Call this function
     * immediately after constructor and, if it returns false, call the
     * destructor.
     * @return true on success, false on failure.
     */
    bool isValid() { return valid; }

    /**
     * Enqueues a packet at list's tail. If it cannot be enqueued is dropped
     * @param packet packet to be enqueued descriptor sctructure
     * @return true if packet can be enqueued, false otherwise
     */
    bool push(packet_header_t& header, const uint8_t* payload);

    packet_header_t getHeader();

    void getData(uint8_t* data);

    /**
     * Removes the packet placed at list's head by avancing it of the
     * packet's size
     */
    void popFront();

    /**
     * @return true if buffer is empty
     */
    bool empty();

private:
    size_t storageSize;
    size_t usedSize;
    uint32_t writeIndex;
    uint32_t readIndex;
    bool valid;
    uint8_t* buffer;

    miosix::FastMutex mutex;

    // Copy constructor and copy assignment are not allowed
    PacketBuffer(PacketBuffer& other);
    PacketBuffer& operator=(const PacketBuffer& other);
};
