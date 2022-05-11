/* Copyright (c) 2019-2022 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron, Davide Mor, Alberto Nidasio
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

#include <drivers/timer/TimestampTimer.h>
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
 * Data can only be appended to the payload. The packet can be marked ready.
 *
 * @tparam len Packet's payload length.
 */
template <unsigned int len>
class Packet
{

public:
    /**
     * @brief Reserves a fixed length for the packet.
     */
    Packet() { content.reserve(len); };

    /**
     * @brief Clears the buffer.
     */
    ~Packet() { content.clear(); };

    /**
     * @brief Append a given message to the packet.
     *
     * If the message can't fit inside the remaining space only the first bytes
     * are copied.
     *
     * @param msg The message to be appended.
     * @param msgLen Length of the message.
     * @return How many bytes were actually appended.
     */
    size_t append(const uint8_t* msg, size_t msgLen);

    /**
     * @brief Mark the packet as ready.
     */
    inline void markAsReady() { ready = true; }

    /**
     * @brief Copies the content of the payload at the given address.
     *
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
    inline uint64_t getTimestamp() const { return timestamp; }

    /**
     * @return The occupied portion of the buffer [bytes].
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
     * @param os For example, std::cout.
     */
    void print(std::ostream& os) const;

    // The buffer itself
    std::vector<uint8_t> content;

private:
    unsigned int msgCounter = 0;
    uint64_t timestamp      = 0;
    bool ready              = false;
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
            timestamp = TimestampTimer::getInstance().getTimestamp();

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
    timestamp  = 0;
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
    os << "timestamp=" << timestamp << ", ready=" << ready
       << ", size=" << content.size() << ", msgCounter=" << msgCounter
       << ", content= ";

    for (auto const& i : content)
        os << i;
    os << '\n';
}

/**
 * @brief A SyncPacketQueue is a SyncCircularBuffer of Packets.
 *
 * The difference is that you pop() Packets but you append() bytes. The bytes
 * will be appended to the first available packet and the next ones.
 * This class is suitable for synchronization between two threads.
 *
 * @tparam pktLen Maximum length of each packet [bytes].
 * @tparam pktNum Total number of packets.
 */
template <unsigned int pktLen, unsigned int pktNum>
class SyncPacketQueue
{
public:
    /**
     * @brief Try to append a given message to the packets queue.
     *
     * The message is appended to the last packet and if the space isn't enough,
     * it is divided into successive packets. If there are no more available
     * packets, the oldest one is overwritten.
     *
     * The message isn't added to the queue only if there is no space
     * considering all the queue packets.
     *
     * @param msg The message to be appended.
     * @param msgLen Length of the message [bytes].
     * @return True if the message was appended.
     */
    bool put(uint8_t* msg, size_t msgLen)
    {
        // Check if the message is empty
        if (msgLen == 0)
            return false;

        // Check if the queue can hold the packet
        if (msgLen > pktLen * pktNum)
            return false;

        {
            // Lock the mutex on the buffer
            Lock<FastMutex> l(mutex);

            // Add an element if there isn't any
            if (buffer.count() == 0)
                buffer.put({});

            // Write all the packet
            while (msgLen > 0)
            {
                // If the last packet is ready append a new one
                if (buffer.last().isReady())
                    buffer.put({});

                // Append what data is possible to the last packet
                size_t appendedLength = buffer.last().append(msg, msgLen);

                // If the packet is full mark it as ready
                if (buffer.last().isFull())
                    buffer.last().markAsReady();

                // Go forward in the data
                msgLen -= appendedLength;
                msg += appendedLength;
            }
        }

        // Wake all waiting threads
        condVerNotEmpty.broadcast();

        return true;
    }

    /**
     * @return The oldest packet, without removing it from the queue.
     */
    const Packet<pktLen>& get()
    {
        Lock<FastMutex> l(mutex);
        return buffer.get();
    }

    /**
     * @return The oldest packet, removing it from the queue.
     */
    const Packet<pktLen>& pop()
    {
        Lock<FastMutex> l(mutex);
        return buffer.pop();
    }

    /**
     * @return True if all the packets have been marked as ready.
     */
    bool isFull()
    {
        Lock<FastMutex> l(mutex);

        if (buffer.count() > 0)
            return buffer.isFull() && buffer.last().isReady();
        else
            return false;
    }

    /**
     * @return True if all the packets are completely empty.
     */
    bool isEmpty()
    {
        Lock<FastMutex> l(mutex);
        return buffer.isEmpty();
    }

    /**
     * @brief Blocks the calling thread until the queue is not empty.
     *
     * Returns immediately if already not empty.
     */
    void waitUntilNotEmpty()
    {
        Lock<FastMutex> l(mutex);
        if (buffer.isEmpty())
            condVerNotEmpty.wait(mutex);
    }

    /**
     * @return The number of packets that are ready to be sent.
     */
    size_t countReady()
    {
        Lock<FastMutex> l(mutex);

        if (!buffer.isEmpty())
            return buffer.last().isReady() ? buffer.count()
                                           : buffer.count() - 1;
        else
            return 0;
    }

    /**
     * @return The number of packets in use, that are either fully or partially
     * filled.
     */
    size_t countNotEmpty()
    {
        Lock<FastMutex> l(mutex);
        return buffer.count();
    }

private:
    FastMutex mutex;
    ConditionVariable condVerNotEmpty;
    CircularBuffer<Packet<pktLen>, pktNum> buffer;
};

}  // namespace Boardcore
