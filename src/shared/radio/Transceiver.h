/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Nuno Barcellos
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

#include <stddef.h>
#include <stdint.h>

namespace Boardcore
{

class Transceiver
{
public:
    Transceiver() {}

    virtual ~Transceiver() {}

    /**
     * @brief Send a packet.
     *
     * The function must block the thread until the packet is sent (successfully
     * or not).
     *
     * @param packet Pointer to the packet (at least packetLength bytes).
     * @param packetLength Length of the packet to be sent.
     * @return True if the message was sent correctly.
     */
    virtual bool send(uint8_t* packet, size_t packetLength) = 0;

    /**
     * @brief Wait until a new packet is received.
     *
     * @param packet Buffer to store the packet (at least packetLength bytes).
     * @param packetLength Maximum length of the received data.
     * @return Size of the data received or -1 if failure
     */
    virtual ssize_t receive(uint8_t* packet, size_t packetLength) = 0;
};

}  // namespace Boardcore
