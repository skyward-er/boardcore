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

#include <functional>

namespace Boardcore
{

/**
 * @brief Class representing an IPv4 ip.
 */
struct WizIp
{
    uint8_t a, b, c, d;
};

/**
 * @brief Class representing an ethernet MAC address.
 */
struct WizMac
{
    uint8_t a, b, c, d, e, f;
};

/**
 * @brief Driver for the WizNet W5500 ethernet.
 */
class Wiz5500
{
public:
    struct PhyState
    {
        bool full_duplex;  //< True if full duplex is enabled, false if link is
                           //only half duplex.
        bool based_100mbps;  //< True if 100Mbps, false if only 10Mpbs.
        bool link_up;        //< True if link is up, false if it is down.
    };

    using OnIpConflictCb      = std::function<void()>;
    using OnDestUnreachableCb = std::function<void(WizIp, uint16_t)>;

    /**
     * @brief Build an instance of the driver.
     *
     * @param bus The underlying SPI bus.
     * @param cs The SPI cs pin.
     * @param intn The INTn pin.
     * @param clock_divider Selected SPI clock divider.
     */
    Wiz5500(SPIBus& bus, miosix::GpioPin cs, miosix::GpioPin intn,
            SPI::ClockDivider clock_divider);
    ~Wiz5500();

    /**
     * @brief Sets the callback to be invoked when the device detects an IP.
     * conflict.
     *
     * WARNING: DO NOT BLOCK IN THIS FUNCTION! ESPECIALLY DO NOT CALL
     * connectTcp, listenTcp, send, recv, recvfrom, close, AS THEY WILL
     * DEADLOCK!
     *
     * This callback will run in the interrupt service routine, and blocking
     * this will block interrupt handling. This is not an issue, but while the
     * thread is blocked no interrupt will be dispatched, and the aforementioned
     * functions will not return.
     *
     * @param cb Callback to be invoked.
     */
    void setOnIpConflict(OnIpConflictCb cb);

    /**
     * @brief Sets the callback to be invoked when the device detects an
     * unreachable host.
     *
     * WARNING: DO NOT BLOCK IN THIS FUNCTION! ESPECIALLY DO NOT CALL
     * connectTcp, listenTcp, send, recv, recvfrom, close, AS THEY WILL
     * DEADLOCK! *DO NOT* CALL close!
     *
     * This callback will run in the interrupt service routine, and blocking
     * this will block interrupt handling. This is not an issue, but while the
     * thread is blocked no interrupt will be dispatched, and the aforementioned
     * functions will not return.
     *
     * @param cb Callback to be invoked.
     */
    void setOnDestUnreachable(OnDestUnreachableCb cb);

    /**
     * @brief Get current PHY state, can be used to poll link status, and wait for link up.
     * 
     * @returns The current PHY state.
    */
    PhyState getPhyState();

    /**
     * @brief Resets the device.
     * Performs a software resets, resetting all registers and closing all
     * sockets. Also checks for hardware presence.
     *
     * @returns False if the device is not connected properly (SPI comunication
     * failure).
     */
    bool reset();

    /**
     * @brief Handle an interrupt from INTn.
     */
    void handleINTn();

    /**
     * @brief Set global gateway ip.
     */
    void setGatewayIp(WizIp ip);

    /**
     * @brief Set global subnet mask.
     */
    void setSubnetMask(WizIp mask);

    /**
     * @brief Set the device MAC address.
     */
    void setSourceMac(WizMac mac);

    /**
     * @brief Set the device IP address.
     */
    void setSourceIp(WizIp ip);

    /**
     * @brief Connect to a remote socket via TCP.
     *
     * @param sock_n Index of the socket, from 0 to 7.
     * @param src_port Local port of the TCP socket.
     * @param dst_ip Remote IP of the TCP socket.
     * @param dst_port Remote port of the TCP socket.
     * @param timeout Timeout for the operation in ms (or -1 if no timeout).
     *
     * @return True in case of success, false otherwise.
     */
    bool connectTcp(int sock_n, uint16_t src_port, WizIp dst_ip,
                    uint16_t dst_port, int timeout = -1);

    /**
     * @brief Listen for a single remote TCP connection.
     *
     * @param sock_n Index of the socket, from 0 to 7.
     * @param src_port Local port of the TCP socket.
     * @param dst_ip Remote IP of the TCP socket.
     * @param dst_port Remote port of the TCP socket.
     * @param timeout Timeout for the operation in ms (or -1 if no timeout).
     *
     * @return True in case of success, false otherwise.
     */
    bool listenTcp(int sock_n, uint16_t src_port, WizIp& dst_ip,
                   uint16_t& dst_port, int timeout = -1);

    /**
     * @brief Open a simple UDP socket.
     *
     * @param sock_n Index of the socket, from 0 to 7.
     * @param src_port Local port of the UDP socket.
     * @param dst_ip Remote IP of the UDP socket.
     * @param dst_port Remote port of the UDP socket.
     * @param timeout Timeout for the operation in ms (or -1 if no timeout).
     *
     * @return True in case of success, false otherwise.
     */
    bool openUdp(int sock_n, uint16_t src_port, WizIp dst_ip, uint16_t dst_port,
                 int timeout = -1);

    /**
     * @brief Send data through the socket (works both in TCP and UDP).
     *
     * @param sock_n Index of the socket, from 0 to 7.
     * @param data Data to be transmitted.
     * @param len Length of the data.
     * @param timeout Timeout for the operation in ms (or -1 if no timeout).
     *
     * @return True in case of success, false otherwise.
     */
    bool send(int sock_n, const uint8_t* data, size_t len, int timeout = -1);

    /**
     * @brief Receive data from the socket (works only in TCP).
     *
     * @param sock_n Index of the socket, from 0 to 7.
     * @param data Buffer to store the data.
     * @param len Maximum length of the data.
     * @param timeout Timeout for the operation in ms (or -1 if no timeout).
     *
     * @return The length of the received data in case of success, -1 otherwise.
     */
    ssize_t recv(int sock_n, uint8_t* data, size_t len, int timeout = -1);

    /**
     * @brief Receive data from the socket (works only in UDP).
     *
     * @param sock_n Index of the socket, from 0 to 7.
     * @param data Buffer to store the data.
     * @param len Maximum length of the data.
     * @param dst_ip Remote IP of the UDP socket.
     * @param dst_port Remote port of the UDP socket.
     * @param timeout Timeout for the operation in ms (or -1 if no timeout).
     *
     * @return The length of the received data in case of success, -1 otherwise.
     */
    ssize_t recvfrom(int sock_n, uint8_t* data, size_t len, WizIp& dst_ip,
                     uint16_t& dst_port, int timeout = -1);

    /**
     * @brief Close a socket.
     *
     * @param sock_n Index of the socket, from 0 to 7.
     * @param timeout Timeout for the operation in ms (or -1 if no timeout).
     */
    void close(int sock_n, int timeout = -1);

private:
    static constexpr int NUM_THREAD_WAIT_INFOS = 16;
    static constexpr int NUM_SOCKETS           = 8;

    void waitForINTn(miosix::Lock<miosix::FastMutex>& l);
    int waitForSocketIrq(miosix::Lock<miosix::FastMutex>& l, int sock_n,
                         uint8_t irq_mask, int timeout);

    void runInterruptServiceRoutine(miosix::Lock<miosix::FastMutex>& l);

    void spiRead(uint8_t block, uint16_t address, uint8_t* data, size_t len);
    void spiWrite(uint8_t block, uint16_t address, const uint8_t* data,
                  size_t len);

    uint8_t spiRead8(uint8_t block, uint16_t address);
    uint16_t spiRead16(uint8_t block, uint16_t address);
    WizIp spiReadIp(uint8_t block, uint16_t address);
    // Avoid stupid linter error
    // WizMac spiReadMac(uint8_t block, uint16_t address);

    void spiWrite8(uint8_t block, uint16_t address, uint8_t data);
    void spiWrite16(uint8_t block, uint16_t address, uint16_t data);
    void spiWriteIp(uint8_t block, uint16_t address, WizIp data);
    void spiWriteMac(uint8_t block, uint16_t address, WizMac data);

    miosix::Thread* interrupt_service_thread = nullptr;

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

    OnIpConflictCb on_ip_conflict;
    OnDestUnreachableCb on_dest_unreachable;

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