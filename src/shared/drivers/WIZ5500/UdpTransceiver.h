/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <radio/Transceiver.h>

#include <memory>

#include "WIZ5500.h"

namespace Boardcore
{

class UdpTransceiver : public Transceiver
{
public:
    UdpTransceiver(std::shared_ptr<Wiz5500> wiz, int sock_n)
        : wiz(std::move(wiz)), sock_n(sock_n)
    {
    }

    ~UdpTransceiver()
    {
        // Put a timeout here because we are in the destructor
        wiz->close(100);
    }

    /**
     * @brief Open this UdpTransceiver on the specified ports.
     * 
     * @param recv_port Port from which to read packets from.
     * @param send_port Port from which to send packets from.
    */
    bool open(uint16_t recv_port, uint16_t send_port)
    {
        return wiz->openUdp(sock_n, src_port, {255, 255, 255, 255}, dst_port);
    }

    /**
     * @brief Close this transceiver.
    */
    void close() { wiz->close(); }

    bool send(uint8_t* pkt, size_t len) { return wiz->send(sock_n, pkt, len); }

    ssize_t receive(uint8_t* pkt, size_t len)
    {
        WizIp dst_ip;
        uint16_t dst_port;
        return wiz->recvfrom(sock_n, pkt, len, dst_ip, dst_port);
    }

private:
    std::shared_ptr<Wiz5500> wiz;
    int sock_n;
}

}  // namespace Boardcore