/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Nuno Barcellos
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

#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <Common.h>

class Transmitter
{
public:
    /*
     * Send a packet. Pure virtual function.
     * @param pkt               Pointer to the packet (needs to be at least pkt_len bytes).
     * @param pkt_len           Lenght of the packet to be sent.
     * @return                  True if the message was sent correctly.
     */
    virtual bool send(const uint8_t* pkt, uint32_t pkt_len) = 0;

    /*
     * Receive a packet. Pure virtual function.
     * @param pkt               Pointer to the buffer (needs to be at least pkt_len bytes).
     * @param pkt_len           Lenght of the packet to be received.
     */
    virtual void receive(uint8_t* pkt, uint32_t pkt_len) = 0;
};

#endif
