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

#include <ActiveObject.h>
#include <drivers/spi/SPIDriver.h>
#include <miosix.h>

namespace Boardcore
{

struct WizIp
{
    uint8_t a, b, c, d;
};

struct WizMac
{
    uint8_t a, b, c, d, e, f;
};

class WizCore : public ActiveObject
{
public:
    WizCore(SPIBus& bus, miosix::GpioPin cs, miosix::GpioPin intn,
            SPI::ClockDivider clock_divider);
    ~WizCore();

    bool start() override;

    void handleINTn();

    void setGatewayIp(WizIp ip);
    void setSubnetMask(WizIp mask);
    void setSourceMac(WizMac mac);
    void setSourceIp(WizIp ip);

    bool connectTcp(int sock_n, uint16_t src_port, WizIp dst_ip,
                    uint16_t dst_port, int timeout = -1);
    bool listenTcp(int sock_n, uint16_t src_port, WizIp &dst_ip, uint16_t &dst_port, int timeout = -1);
    bool openUdp(int sock_n, uint16_t src_port, WizIp dst_ip, uint16_t dst_port,
                 int timeout = -1);
    bool send(int sock_n, const uint8_t* data, size_t len, int timeout = -1);
    ssize_t recv(int sock_n, uint8_t* data, size_t len, int timeout = -1);
    ssize_t recvfrom(int sock_n, uint8_t* data, size_t len, WizIp& dst_ip,
                     uint16_t& dst_port, int timeout = -1);
    void close(int sock_n, int timeout = -1);

private:
    static constexpr int NUM_THREAD_WAIT_INFOS = 16;
    static constexpr int NUM_SOCKETS           = 8;

    void run() override;

    void waitForINTn(miosix::Lock<miosix::FastMutex>& l);
    int waitForSocketIrq(miosix::Lock<miosix::FastMutex>& l, int sock_n,
                         uint8_t irq_mask, int timeout);

    void spiRead(uint8_t block, uint16_t address, uint8_t* data, size_t len);
    void spiWrite(uint8_t block, uint16_t address, const uint8_t* data,
                  size_t len);

    uint8_t spiRead8(uint8_t block, uint16_t address);
    uint16_t spiRead16(uint8_t block, uint16_t address);
    WizIp spiReadIp(uint8_t block, uint16_t address);
    WizMac spiReadMac(uint8_t block, uint16_t address);

    void spiWrite8(uint8_t block, uint16_t address, uint8_t data);
    void spiWrite16(uint8_t block, uint16_t address, uint16_t data);
    void spiWriteIp(uint8_t block, uint16_t address, WizIp data);
    void spiWriteMac(uint8_t block, uint16_t address, WizMac data);

    struct ThreadWaitInfo
    {
        int sock_n;
        uint8_t irq_mask;
        uint8_t irq;
        miosix::Thread* thread;
    };

    enum class SocketMode
    {
        TCP,
        UDP,
        CLOSED
    };

    struct SocketInfo
    {
        SocketMode mode;
        int irq_mask;
    };

    SocketInfo socket_infos[NUM_SOCKETS];
    ThreadWaitInfo wait_infos[NUM_THREAD_WAIT_INFOS];

    miosix::GpioPin intn;
    miosix::FastMutex mutex;
    SPISlave slave;
};

}  // namespace Boardcore

namespace std
{
inline ostream& operator<<(ostream& os, const Boardcore::WizIp& ip)
{
    auto old_flags = os.flags(os.dec);
    os << (int)ip.a << "." << (int)ip.b << "." << (int)ip.c << "." << (int)ip.d;
    os.flags(old_flags);
    return os;
}

inline ostream& operator<<(ostream& os, const Boardcore::WizMac& mac)
{
    auto old_flags = os.flags(os.hex);
    os << (int)mac.a << ":" << (int)mac.b << ":" << (int)mac.c << ":"
       << (int)mac.d << ":" << (int)mac.e << ":" << (int)mac.f;
    os.flags(old_flags);
    return os;
}
}  // namespace std