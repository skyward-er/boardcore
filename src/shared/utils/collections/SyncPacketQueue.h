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

#include <cstdint>
#include <list>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "CircularBuffer.h"

// This header can be compiled to run on a PC, for easier testing.
#ifdef COMPILE_FOR_X86
#warning The flag COMPILE_FOR_X86 is active! If this is flight code, shame on you
#define TRACE(x) printf(x)
#define MIOSIX_ONLY(x)
#else
#define MIOSIX_ONLY(x) x
#include "Debug.h"
#include "miosix.h"

using miosix::ConditionVariable;
using miosix::FastMutex;
using miosix::Lock;
using std::range_error;
#endif

namespace Boardcore
{

/*******************************************************************************
 * @brief The Packet class is used for packing together messages with variable
 * lengths into a fixed size packet. Useful for telemetry.
 * The buffer can only be appended, read or flushed. The caller can also mark
 * the packet as ready to be sent.
 *
 * @tparam len  Maximum length of this packet.
 *******************************************************************************/
template <unsigned int len>
class Packet
{

public:
    /**
     * @brief Constructor: reserves a fixed length for the packet.
     */
    Packet() : msg_count(0), ts(0), ready(false) { content.reserve(len); };

    /**
     * @brief Destructor: clears the buffer.
     */
    ~Packet() { content.clear(); };

    /**
     * @brief Try to append a given message to the packet. If it's the first
     * message, also set the timestamp.
     * @param msg      the message to be appended
     * @param msg_len  length of msg
     * @return true    if the message was appended correctly
     * @return false   if there isn't enough space for the message
     */
    bool tryAppend(const uint8_t* msg, const size_t msg_len);

    /**
     * @brief mark the packet as ready to be sent.
     */
    inline void markAsReady() { ready = true; }

    /**
     * @brief Copies the content of the buffer at a given address.
     * @param buf  Where to copy the content.
     * @return     How many bytes where copied, i.e. the size of the packet.
     */
    size_t dump(uint8_t* buf);

    /**
     * @brief clear the buffer and reset members.
     */
    void clear();

    /**
     * @return true if the packet has reached the maximum length.
     */
    inline bool isFull() const { return content.size() == len; }

    /**
     * @return true if no message has been inserted yet.
     */
    inline bool isEmpty() const { return content.size() == 0; }

    /**
     * @return true if the packet has been marked as ready.
     */
    inline bool isReady() const { return ready; }

    /**
     * @return the timestamp of the first successful call to tryAppend().
     */
    inline uint64_t timestamp() const { return ts; }

    /**
     * @return the occupied portion of the buffer (bytes).
     */
    inline size_t size() const { return content.size(); }

    /**
     * @return the maximum number of bytes in the buffer.
     */
    inline unsigned int maxSize() const { return len; }

    /**
     * @return how many times the tryAppend() function has been called
     * successfully.
     */
    inline unsigned int msgCount() const { return msg_count; }

    /**
     * @brief Print information about this object.
     * @param os  for example, std::cout
     */
    void print(std::ostream& os) const;

    // The buffer itself
    std::vector<uint8_t> content;

private:
    // Helper variables
    unsigned int msg_count;
    uint64_t ts;
    bool ready;
};

template <unsigned int len>
bool Packet<len>::tryAppend(const uint8_t* msg, const size_t msg_len)
{
    if (msg_len == 0 || content.size() + msg_len > len)
    {
        return false;
    }
    else
    {
        // Set the packet's timestamp when the first message is inserted
        if (content.size() == 0)
        {
#ifdef COMPILE_FOR_X86
            ts = time(NULL);
#else
            ts = miosix::getTick();
#endif
        }

        // Append the message to the packet
        content.insert(content.end(), msg, msg + msg_len);
        msg_count++;

        return true;
    }
}

template <unsigned int len>
void Packet<len>::clear()
{
    content.clear();
    msg_count = 0;
    ts        = 0;
    ready     = false;
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
       << ", size=" << content.size() << ", msgCount=" << msg_count
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
 * @tparam pkt_len  Maximum length of each packet. (bytes)
 * @tparam pkt_num  Total number of packets.
 ******************************************************************************/
template <unsigned int pkt_len, unsigned int pkt_num>
class SyncPacketQueue
{
    using Pkt = Packet<pkt_len>;

public:
    /**
     * @brief Try to append a given message to the last packet. If there isn't
     * enough space, the packet is marked as ready and the message is appended
     * to the next packet. If there are no more avaible packets, the oldest one
     * is overwritten.
     * @param msg      the message to be appended
     * @param msg_len  length of msg
     * @return true    if the message was appended correctly
     * @return false   if there isn't enough space for the message
     */
    int put(uint8_t* msg, size_t msg_len)
    {
        int dropped = 0;

        if (msg_len == 0 || msg_len > pkt_len)
        {
            return -1;
        }

        {
            MIOSIX_ONLY(Lock<FastMutex> l(mutex);)
            // Add an element if there isn't any
            if (buffer.count() == 0)
            {
                buffer.put(Pkt{});
            }

            Pkt& last = buffer.last();

            bool added = false;

            // Add to the current packet only if it isn't already ready
            if (!last.isReady())
            {
                added = last.tryAppend(msg, msg_len);
            }

            // If already ready or cannot fit the new data, add to a new packet
            if (!added)
            {
                // Mark the packet as ready (in the case it wasn't already)
                last.markAsReady();

                if (buffer.isFull())
                {
                    // We have dropped a packet
                    ++dropped;
                }
                // Add a new packet and fill that instead
                Pkt& newpkt = buffer.put(Pkt{});

                if (!newpkt.tryAppend(msg, msg_len))
                {
                    TRACE("Packet is too big!\n");
                    return -1;
                }
            }

            // Mark as ready if the packet is full
            if (buffer.last().isFull())
            {
                buffer.last().markAsReady();
            }

            MIOSIX_ONLY(cv_notempty.broadcast();)
            return dropped;
        }
    }

    /**
     * @return a copy of the oldest packet, without removing it from the queue.
     */
    const Pkt& get()
    {
        MIOSIX_ONLY(Lock<FastMutex> l(mutex);)
        return buffer.get();
    }

    /**
     * @return the oldest packet, removing it from the queue.
     */
    const Pkt& pop()
    {
        MIOSIX_ONLY(Lock<FastMutex> l(mutex);)
        return buffer.pop();
    }

    /**
     * @return true if all the packets have been marked as ready.
     */
    bool isFull()
    {
        MIOSIX_ONLY(Lock<FastMutex> l(mutex);)

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
        MIOSIX_ONLY(Lock<FastMutex> l(mutex);)

        return buffer.isEmpty();
    }

    /**
     * @brief Blocks the calling thread until the queue is not empty.
     * Returns immediately if already not empty.
     */
    void waitUntilNotEmpty()
    {
#ifndef COMPILE_FOR_X86
        Lock<FastMutex> l(mutex);
        if (buffer.isEmpty())
        {
            cv_notempty.wait(mutex);
        }
#endif
    }

    /**
     * @return the number of packets that are ready to be sent.
     */
    size_t countReady()
    {
        MIOSIX_ONLY(Lock<FastMutex> l(mutex);)

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
        MIOSIX_ONLY(Lock<FastMutex> l(mutex);)

        return buffer.count();
    }

private:
    MIOSIX_ONLY(FastMutex mutex;)
    MIOSIX_ONLY(ConditionVariable cv_notempty;)

    CircularBuffer<Pkt, pkt_num> buffer;
};

}  // namespace Boardcore
