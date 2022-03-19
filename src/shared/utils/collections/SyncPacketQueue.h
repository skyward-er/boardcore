/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Alvise de'Faveri Tron
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
#include <utils/Debug.h>

#include <cstdint>
#include <list>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "CircularBuffer.h"

using miosix::ConditionVariable;
using miosix::FastMutex;
using miosix::Lock;
using std::range_error;

namespace Boardcore
{

/**
 * @brief The Packet class is used for packing together messages with variable
 * lengths into a fixed size packet. Useful for telemetry.
 *
 * The buffer can only be appended, read or flushed. The caller can also mark
 * the packet as ready to be sent.
 *
 * @tparam len Maximum length for the packet.
 */
template <unsigned int len>
class Packet
{

public:
    /**
     * @brief Reserves a fixed length for the packet.
     */
    Packet() : msgCounter(0), ts(0), ready(false) { content.reserve(len); };

    /**
     * @brief Clears the buffer.
     */
    ~Packet() { content.clear(); };

    /**
     * @brief Append a given message to the packet.
     *
     * If it's the first message, also set the timestamp.
     *
     * @param msg The message to be appended.
     * @param msgLen Length of msg.
     * @return How many bytes were actually appended.
     */
    size_t append(const uint8_t* msg, size_t msgLen);

    /**
     * @brief Mark the packet as ready to be sent.
     */
    inline void markAsReady() { ready = true; }

    /**
     * @brief Copies the content of the buffer at a given address.
     * @param buf Where to copy the content.
     * @return How many bytes where copied, i.e. the size of the packet.
     */
    size_t dump(uint8_t* buf);

    /**
     * @brief Clear the buffer and reset members.
     */
    void clear();

    /**
     * @return True if the packet has reached the maximum length.
     */
    inline bool isFull() const { return content.size() == len; }

    /**
     * @return True if no message has been inserted yet.
     */
    inline bool isEmpty() const { return content.size() == 0; }

    /**
     * @return True if the packet has been marked as ready.
     */
    inline bool isReady() const { return ready; }

    /**
     * @return The timestamp of the first successful call to append().
     */
    inline uint64_t timestamp() const { return ts; }

    /**
     * @return The occupied portion of the buffer (bytes).
     */
    inline size_t size() const { return content.size(); }

    /**
     * @return The maximum number of bytes in the buffer.
     */
    inline unsigned int maxSize() const { return len; }

    /**
     * @return How many times the append() function has been called
     * successfully.
     */
    inline unsigned int getMsgCount() const { return msgCounter; }

    /**
     * @brief Print information about this object.
     *
     * @param os For example, std::cout
     */
    void print(std::ostream& os) const;

    // The buffer itself
    std::vector<uint8_t> content;

private:
    unsigned int msgCounter;
    uint64_t ts;
    bool ready;
};

template <unsigned int len>
size_t Packet<len>::append(const uint8_t* msg, size_t msgLen)
{
    size_t remaining = len - content.size();
    msgLen           = std::min(remaining, msgLen);

    if (msgLen != 0)
    {
        // Set the packet's timestamp when the first message is inserted
        if (content.size() == 0)
        {
            ts = miosix::getTick();
        }

        // Append the message to the packet
        content.insert(content.end(), msg, msg + msgLen);
        msgCounter++;
    }

    return msgLen;
}

template <unsigned int len>
void Packet<len>::clear()
{
    content.clear();
    msgCounter = 0;
    ts         = 0;
    ready      = false;
}

template <unsigned int len>
size_t Packet<len>::dump(uint8_t* buf)
{
    std::copy(content.begin(), content.end(), buf);
    return content.size();
}

template <unsigned int len>
void Packet<len>::print(std::ostream& os) const
{
    os << "timestamp=" << ts << ", ready=" << ready
       << ", size=" << content.size() << ", msgCounter=" << msgCounter
       << ", content= ";

    for (auto const& i : content)
    {
        os << i;
    }
    os << '\n';
}

/******************************************************************************
 * @brief A SyncPacketQueue is a SyncCircularBuffer of Packets. The difference
 * is that you pop() Packets but you append() bytes. The bytes will be appended
 * to the first available packet. This class is suitable for synchronization
 * between two threads.
 *
 * @tparam pktLen  Maximum length of each packet. (bytes)
 * @tparam pktNum  Total number of packets.
 ******************************************************************************/
template <unsigned int pktLen, unsigned int pktNum>
class SyncPacketQueue
{
    using Pkt = Packet<pktLen>;

public:
    /**
     * @brief Try to append a given message to the last packet. If there isn't
     * enough space, the packet is marked as ready and the message is appended
     * to the next packet. If there are no more available packets, the oldest
     * one is overwritten.
     *
     * @param msg      the message to be appended
     * @param msgLen  length of msg
     * @return true    if the message was appended correctly
     * @return false   if there isn't enough space for the message
     */
    int put(uint8_t* msg, size_t msgLen)
    {
        int dropped = 0;

        if (msgLen == 0)
        {
            return -1;
        }

        {
            Lock<FastMutex> l(mutex);
            // Add an element if there isn't any
            if (buffer.count() == 0)
            {
                buffer.put(Pkt{});
            }

            while (msgLen > 0)
            {
                if (buffer.last().isReady())
                {
                    if (buffer.isFull())
                    {
                        // We have dropped a packet
                        ++dropped;
                    }

                    // If the last pkt is ready, append a new one
                    buffer.put(Pkt{});
                    // FIXME(davide.mor): Figure out quantum shenanigans
                    // uncommenting the following line causes everything to
                    // break, why?

                    // last = buffer.last();
                }

                size_t sentLen = buffer.last().append(msg, msgLen);

                msgLen -= sentLen;
                msg += sentLen;

                // Mark as ready if the packet is full
                if (buffer.last().isFull())
                {
                    buffer.last().markAsReady();
                }
            }

            cvNotempty.broadcast();
            return dropped;
        }
    }

    /**
     * @return a copy of the oldest packet, without removing it from the queue.
     */
    const Pkt& get()
    {
        Lock<FastMutex> l(mutex);
        return buffer.get();
    }

    /**
     * @return the oldest packet, removing it from the queue.
     */
    const Pkt& pop()
    {
        Lock<FastMutex> l(mutex);
        return buffer.pop();
    }

    /**
     * @return true if all the packets have been marked as ready.
     */
    bool isFull()
    {
        Lock<FastMutex> l(mutex);

        if (buffer.count() > 0)
        {
            return buffer.isFull() && buffer.last().isReady();
        }
        else
        {
            return false;
        }
    }

    /**
     * @return true if all the packets are completely empty.
     */
    bool isEmpty()
    {
        Lock<FastMutex> l(mutex);

        return buffer.isEmpty();
    }

    /**
     * @brief Blocks the calling thread until the queue is not empty.
     * Returns immediately if already not empty.
     */
    void waitUntilNotEmpty()
    {
        Lock<FastMutex> l(mutex);
        if (buffer.isEmpty())
        {
            cvNotempty.wait(mutex);
        }
    }

    /**
     * @return the number of packets that are ready to be sent.
     */
    size_t countReady()
    {
        Lock<FastMutex> l(mutex);

        if (!buffer.isEmpty())
        {
            return buffer.last().isReady() ? buffer.count()
                                           : buffer.count() - 1;
        }
        else
        {
            return 0;
        }
    }

    /**
     * @return the number of packets in use, that are either fully or partially
     * filled.
     */
    size_t countNotEmpty()
    {
        Lock<FastMutex> l(mutex);

        return buffer.count();
    }

private:
    FastMutex mutex;
    ConditionVariable cvNotempty;

    CircularBuffer<Pkt, pktNum> buffer;
};

}  // namespace Boardcore
