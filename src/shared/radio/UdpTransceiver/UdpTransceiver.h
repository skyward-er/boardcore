/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto
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

#include <drivers/WIZ5500/WIZ5500.h>
#include <radio/Transceiver.h>

#include <memory>

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
        wiz->close(sock_n, 100);
    }

    bool send(uint8_t* pkt, size_t len) { return wiz->send(sock_n, pkt, len); }

    ssize_t receive(uint8_t* pkt, size_t len)
    {
        WizIp dst_ip;
        uint16_t dst_port;
        return wiz->recvfrom(sock_n, pkt, len, dst_ip, dst_port);
    }

    /**
     * @brief Open this UdpTransceiver on the specified ports.
     *
     * @param recv_port Port to receive data from.
     * @param send_ip IP to send data to.
     * @param send_port Port to send data to.
     */
    bool open(uint16_t recv_port, Boardcore::WizIp send_ip, uint16_t send_port)
    {
        return wiz->openUdp(sock_n, recv_port, send_ip, send_port);
    }

    /**
     * @brief Close this transceiver.
     */
    void close() { wiz->close(sock_n); }

private:
    std::shared_ptr<Wiz5500> wiz;
    int sock_n;
};

}  // namespace Boardcore
