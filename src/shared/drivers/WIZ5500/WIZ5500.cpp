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

#include "WIZ5500.h"

#include <drivers/interrupt/external_interrupts.h>
#include <interfaces/endianness.h>
#include <kernel/scheduler/scheduler.h>

#include "WIZ5500Defs.h"

using namespace Boardcore;
using namespace miosix;

// Keep a 500ms timeout even for the general INTn
constexpr long long INTN_TIMEOUT = 500;

SPIBusConfig getSpiBusConfig(SPI::ClockDivider clock_divider)
{
    SPIBusConfig bus_config = {};
    bus_config.clockDivider = clock_divider;
    bus_config.mode         = SPI::Mode::MODE_0;
    bus_config.bitOrder     = SPI::Order::MSB_FIRST;
    bus_config.byteOrder    = SPI::Order::MSB_FIRST;

    return bus_config;
}

Wiz5500::Wiz5500(SPIBus &bus, miosix::GpioPin cs, miosix::GpioPin intn,
                 SPI::ClockDivider clock_divider)
    : intn(intn), slave(bus, cs, getSpiBusConfig(clock_divider))
{
    enableExternalInterrupt(intn, InterruptTrigger::FALLING_EDGE);

    // Reset thread wait infos
    for (int i = 0; i < NUM_THREAD_WAIT_INFOS; i++)
    {
        wait_infos[i].sock_n   = -1;
        wait_infos[i].irq_mask = 0;
        wait_infos[i].irq      = 0;
        wait_infos[i].thread   = nullptr;
    }

    // Reset socket infos
    for (int i = 0; i < NUM_SOCKETS; i++)
    {
        socket_infos[i].mode     = Wiz5500::SocketMode::CLOSED;
        socket_infos[i].irq_mask = 0;
    }
}

Wiz5500::~Wiz5500() { disableExternalInterrupt(intn); }

void Wiz5500::setOnIpConflict(OnIpConflictCb cb)
{
    Lock<FastMutex> l(mutex);
    on_ip_conflict = cb;
}

void Wiz5500::setOnDestUnreachable(OnDestUnreachableCb cb)
{
    Lock<FastMutex> l(mutex);
    on_dest_unreachable = cb;
}

bool Wiz5500::reset()
{
    Lock<FastMutex> l(mutex);

    // First check that the device is actually present
    if (spiRead8(0, Wiz::Common::REG_VERSIONR) != Wiz::VERSION)
    {
        return false;
    }

    // Perform a software reset
    spiWrite8(0, Wiz::Common::REG_MR, 1 << 7);
    // Enable all socket interrupts
    spiWrite8(0, Wiz::Common::REG_SIMR, 0b11111111);
    spiWrite8(0, Wiz::Common::REG_IMR, 0b11110000);

    // Reset all socketsOSI
    for (int i = 0; i < NUM_SOCKETS; i++)
    {
        spiWrite8(Wiz::getSocketRegBlock(i), Wiz::Socket::REG_MR, 0);
    }

    return true;
}

void Wiz5500::handleINTn()
{
    if (interrupt_service_thread)
    {
        interrupt_service_thread->IRQwakeup();
        if (interrupt_service_thread->IRQgetPriority() >
            miosix::Thread::IRQgetCurrentThread()->IRQgetPriority())
        {
            miosix::Scheduler::IRQfindNextThread();
        }
    }
}

void Wiz5500::setGatewayIp(WizIp ip)
{
    Lock<FastMutex> l(mutex);
    spiWriteIp(0, Wiz::Common::REG_GAR, ip);
}

void Wiz5500::setSubnetMask(WizIp mask)
{
    Lock<FastMutex> l(mutex);
    spiWriteIp(0, Wiz::Common::REG_SUBR, mask);
}

void Wiz5500::setSourceMac(WizMac mac)
{
    Lock<FastMutex> l(mutex);
    spiWriteMac(0, Wiz::Common::REG_SHAR, mac);
}

void Wiz5500::setSourceIp(WizIp ip)
{
    Lock<FastMutex> l(mutex);
    spiWriteIp(0, Wiz::Common::REG_SIPR, ip);
}

bool Wiz5500::connectTcp(int sock_n, uint16_t src_port, WizIp dst_ip,
                         uint16_t dst_port, int timeout)
{
    Lock<FastMutex> l(mutex);

    // Check that we are closed
    if (socket_infos[sock_n].mode != Wiz5500::SocketMode::CLOSED)
    {
        return false;
    }

    // Setup the socket
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_MR,
              Wiz::Socket::buildModeTcp(false));
    spiWrite16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_PORT, src_port);
    spiWriteIp(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_DIPR, dst_ip);
    spiWrite16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_DPORT,
               dst_port);

    // Open the socket
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_CR,
              Wiz::Socket::CMD_OPEN);

    // Ok now check that we actually went into that mode
    int status = spiRead8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_SR);
    if (status != Wiz::Socket::STAT_INIT)
    {
        return false;
    }

    // Connect the socket
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_CR,
              Wiz::Socket::CMD_CONNECT);

    // Ok now wait for either a connection, or a disconnection
    int irq = waitForSocketIrq(
        l, sock_n, Wiz::Socket::Irq::CON | Wiz::Socket::Irq::DISCON, timeout);

    // Connection failed
    if ((irq & Wiz::Socket::Irq::CON) == 0)
    {
        return false;
    }

    socket_infos[sock_n].mode = Wiz5500::SocketMode::TCP;
    return true;
}

bool Wiz5500::listenTcp(int sock_n, uint16_t src_port, WizIp &dst_ip,
                        uint16_t &dst_port, int timeout)
{
    Lock<FastMutex> l(mutex);

    // Check that we are closed
    if (socket_infos[sock_n].mode != Wiz5500::SocketMode::CLOSED)
    {
        return false;
    }

    // Setup the socket
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_MR,
              Wiz::Socket::buildModeTcp(false));
    spiWrite16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_PORT, src_port);

    // Open the socket
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_CR,
              Wiz::Socket::CMD_OPEN);

    // Ok now check that we actually went into that mode
    int status = spiRead8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_SR);
    if (status != Wiz::Socket::STAT_INIT)
    {
        return false;
    }

    // Connect the socket
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_CR,
              Wiz::Socket::CMD_LISTEN);

    // Ok now wait for either a connection, or a disconnection
    int irq =
        waitForSocketIrq(l, sock_n,
                         Wiz::Socket::Irq::CON | Wiz::Socket::Irq::DISCON |
                             Wiz::Socket::Irq::TIMEOUT,
                         timeout);

    // Connection failed
    if ((irq & Wiz::Socket::Irq::CON) == 0)
    {
        return false;
    }

    // Read remote side infos
    dst_ip = spiReadIp(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_DIPR);
    dst_port =
        spiRead16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_DPORT);

    socket_infos[sock_n].mode = Wiz5500::SocketMode::TCP;
    return true;
}

bool Wiz5500::openUdp(int sock_n, uint16_t src_port, WizIp dst_ip,
                      uint16_t dst_port, int timeout)
{
    Lock<FastMutex> l(mutex);

    // Check that we are closed
    if (socket_infos[sock_n].mode != Wiz5500::SocketMode::CLOSED)
    {
        return false;
    }

    // Setup the socket
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_MR,
              Wiz::Socket::buildModeUdp(false, false, false, false));
    spiWrite16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_PORT, src_port);
    spiWriteIp(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_DIPR, dst_ip);
    spiWrite16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_DPORT,
               dst_port);

    // Open the socket
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_CR,
              Wiz::Socket::CMD_OPEN);

    // Ok now check that we actually went into that mode
    int status = spiRead8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_SR);
    if (status != Wiz::Socket::STAT_UDP)
    {
        return false;
    }

    socket_infos[sock_n].mode = Wiz5500::SocketMode::UDP;
    return true;
}

bool Wiz5500::send(int sock_n, const uint8_t *data, size_t len, int timeout)
{
    Lock<FastMutex> l(mutex);

    // We cannot send through a closed socket
    if (socket_infos[sock_n].mode == Wiz5500::SocketMode::CLOSED)
    {
        return false;
    }

    // First get the start write address
    uint16_t start_addr =
        spiRead16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_TX_WR);

    // Fill the TX buffer and update the counter
    spiWrite(Wiz::getSocketTxBlock(sock_n), start_addr, data, len);
    spiWrite16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_TX_WR,
               start_addr + len);

    // Finally tell the device to send the data
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_CR,
              Wiz::Socket::CMD_SEND);

    // Wait for the device to signal a correct send
    int irq = waitForSocketIrq(l, sock_n, Wiz::Socket::Irq::SEND_OK, timeout);

    // It didn't signal it as an error
    if ((irq & Wiz::Socket::Irq::SEND_OK) == 0)
    {
        return false;
    }

    return true;
}

ssize_t Wiz5500::recv(int sock_n, uint8_t *data, size_t len, int timeout)
{
    Lock<FastMutex> l(mutex);

    // This is only valid for TCP
    if (socket_infos[sock_n].mode != Wiz5500::SocketMode::TCP)
    {
        return -1;
    }

    // Wait for the device to signal that we received something, or a
    // disconnection
    int irq = waitForSocketIrq(
        l, sock_n, Wiz::Socket::Irq::RECV | Wiz::Socket::Irq::DISCON, timeout);
    if ((irq & Wiz::Socket::Irq::RECV) == 0)
    {
        return -1;
    }

    // Ok we received something, get the received length
    uint16_t recv_len =
        spiRead16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_RX_RSR);
    uint16_t addr =
        spiRead16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_RX_RD);

    // Check if we actually have space
    if (recv_len < len)
    {
        spiRead(Wiz::getSocketRxBlock(sock_n), addr, data, recv_len);
    }

    addr += recv_len;
    spiWrite16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_RX_RD, addr);

    // Finally tell the device that we correctly received and read the data
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_CR,
              Wiz::Socket::CMD_RECV);

    return recv_len < len ? recv_len : -1;
}

ssize_t Wiz5500::recvfrom(int sock_n, uint8_t *data, size_t len, WizIp &dst_ip,
                          uint16_t &dst_port, int timeout)
{
    Lock<FastMutex> l(mutex);

    // This is only valid for UDP
    if (socket_infos[sock_n].mode != Wiz5500::SocketMode::UDP)
    {
        return -1;
    }

    // Wait for the device to signal that we received something, or a
    // disconnection
    int irq = waitForSocketIrq(
        l, sock_n, Wiz::Socket::Irq::RECV | Wiz::Socket::Irq::DISCON, timeout);
    if ((irq & Wiz::Socket::Irq::RECV) == 0)
    {
        return -1;
    }

    // Ok we received something, get the received length
    uint16_t recv_len =
        spiRead16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_RX_RSR);
    uint16_t addr =
        spiRead16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_RX_RD);

    // First read the internal header
    spiRead(Wiz::getSocketRxBlock(sock_n), addr,
            reinterpret_cast<uint8_t *>(&dst_ip), sizeof(WizIp));
    addr += sizeof(WizIp);
    spiRead(Wiz::getSocketRxBlock(sock_n), addr,
            reinterpret_cast<uint8_t *>(&dst_port), sizeof(uint16_t));
    addr += sizeof(uint16_t);

    // Now, what's this?
    uint16_t what = 0;
    spiRead(Wiz::getSocketRxBlock(sock_n), addr,
            reinterpret_cast<uint8_t *>(&what), sizeof(uint16_t));
    addr += sizeof(uint16_t);

    // Remove what we have already read.
    recv_len -= sizeof(WizIp) + sizeof(uint16_t) + sizeof(uint16_t);

    // Check if we actually have space
    if (recv_len < len)
    {
        spiRead(Wiz::getSocketRxBlock(sock_n), addr, data, recv_len);
    }

    addr += recv_len;
    spiWrite16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_RX_RD, addr);

    // Finally tell the device that we correctly received and read the data
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_CR,
              Wiz::Socket::CMD_RECV);

    return recv_len < len ? recv_len : -1;
}

void Wiz5500::close(int sock_n, int timeout)
{
    Lock<FastMutex> l(mutex);

    // We cannot receive close a closed socket
    if (socket_infos[sock_n].mode == Wiz5500::SocketMode::CLOSED)
    {
        return;
    }

    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_CR,
              Wiz::Socket::CMD_DISCON);

    waitForSocketIrq(l, sock_n, Wiz::Socket::Irq::DISCON, timeout);
    socket_infos[sock_n].mode = Wiz5500::SocketMode::CLOSED;
}

void Wiz5500::waitForINTn(Lock<FastMutex> &l)
{
    long long start        = getTick();
    TimedWaitResult result = TimedWaitResult::NoTimeout;

    Unlock<FastMutex> ul(l);
    FastInterruptDisableLock il;
    while (intn.value() != 0 && result == TimedWaitResult::NoTimeout)
    {
        result = Thread::IRQenableIrqAndTimedWaitMs(il, start + INTN_TIMEOUT);
    }
}

int Wiz5500::waitForSocketIrq(miosix::Lock<miosix::FastMutex> &l, int sock_n,
                              uint8_t irq_mask, int timeout)
{
    // Check that someone else isn't already waiting for one of ours interrupts
    if ((socket_infos[sock_n].irq_mask & irq_mask) != 0)
    {
        return 0;
    }

    // Enable the interrupts requested, updating the IRQ mask
    socket_infos[sock_n].irq_mask |= irq_mask;
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_IMR,
              socket_infos[sock_n].irq_mask);

    Thread *this_thread = Thread::getCurrentThread();

    // Find a free slot in the data structure
    int i = 0;
    while (i < NUM_THREAD_WAIT_INFOS)
    {
        if (wait_infos[i].sock_n == -1)
        {
            wait_infos[i].sock_n   = sock_n;
            wait_infos[i].irq_mask = irq_mask;
            wait_infos[i].irq      = 0;
            wait_infos[i].thread   = this_thread;
            break;
        }

        i++;
    }

    // We didn't find any, return with failure
    if (i == NUM_THREAD_WAIT_INFOS)
    {
        return 0;
    }

    long long start        = getTick();
    TimedWaitResult result = TimedWaitResult::NoTimeout;

    if (interrupt_service_thread != nullptr)
    {
        // There is already someone managing interrupts for us, just wait

        Unlock<FastMutex> ul(l);
        FastInterruptDisableLock il;
        while (wait_infos[i].irq == 0 && result == TimedWaitResult::NoTimeout &&
               interrupt_service_thread != this_thread)
        {
            if (timeout != -1)
            {
                result =
                    Thread::IRQenableIrqAndTimedWaitMs(il, start + timeout);
            }
            else
            {
                Thread::IRQenableIrqAndWait(il);
            }
        }
    }
    else
    {
        // Nobody is managing interrupts, we are doing it ourself
        FastInterruptDisableLock il;
        interrupt_service_thread = this_thread;
    }

    while (interrupt_service_thread == this_thread)
    {
        // Run a single step of the ISR
        runInterruptServiceRoutine(l);

        // Check if we woke up ourself, then we need to elect a new interrupt
        // service thread
        if (wait_infos[i].irq != 0)
        {
            for (int j = 0; j < NUM_THREAD_WAIT_INFOS; j++)
            {
                if (wait_infos[j].irq == 0)
                {
                    {
                        FastInterruptDisableLock il;
                        interrupt_service_thread = wait_infos[j].thread;
                    }

                    wait_infos[j].thread->wakeup();
                    break;
                }
            }
        }
    }

    // The interrupt arrived, clear the slot
    wait_infos[i].sock_n = -1;

    // Disable the interrupts
    socket_infos[sock_n].irq_mask &= ~irq_mask;
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_IMR,
              socket_infos[sock_n].irq_mask);

    return wait_infos[i].irq;
}

void Wiz5500::runInterruptServiceRoutine(Lock<FastMutex> &l)
{
    // Other threads might wake us up in order to
    waitForINTn(l);

    // Ok something happened!

    // First check for general interrupts
    uint8_t ir = spiRead8(0, Wiz::Common::REG_IR);
    spiWrite8(0, Wiz::Common::REG_IR, ir);

    uint8_t sn_ir[NUM_SOCKETS] = {0};

    // Then check for interrupt on all the sockets
    uint8_t sir = spiRead8(0, Wiz::Common::REG_SIR);
    for (int i = 0; i < NUM_SOCKETS; i++)
    {
        if (sir & (1 << i))
        {
            sn_ir[i] = spiRead8(Wiz::getSocketRegBlock(i), Wiz::Socket::REG_IR);
            spiWrite8(Wiz::getSocketRegBlock(i), Wiz::Socket::REG_IR, sn_ir[i]);
        }
    }

    // Ok now wake up all threads in sleep
    for (int i = 0; i < NUM_THREAD_WAIT_INFOS; i++)
    {
        if (wait_infos[i].sock_n != -1)
        {
            int sock_n = wait_infos[i].sock_n;
            int irq    = sn_ir[sock_n] & wait_infos[i].irq_mask;

            if (irq != 0)
            {
                wait_infos[i].irq = irq;
                wait_infos[i].thread->wakeup();
            }
        }
    }

    // Dispatch generic interurpts
    if(ir & Wiz::Common::Irq::CONFLICT) {
        auto cb = on_ip_conflict;
        if(cb) {
            Unlock<FastMutex> ul(l);
            cb();
        }
    }

    if(ir & Wiz::Common::Irq::UNREACH) {
        auto cb = on_dest_unreachable;
        if(cb) {
            WizIp ip = spiReadIp(0, Wiz::Common::REG_UIPR);
            uint16_t port = spiRead16(0, Wiz::Common::REG_UPORTR);

            Unlock<FastMutex> ul(l);
            cb(ip, port);
        }
    }
}

void Wiz5500::spiRead(uint8_t block, uint16_t address, uint8_t *data,
                      size_t len)
{
    // Do a manual SPI transaction
    slave.bus.configure(slave.config);

    slave.bus.select(slave.cs);
    slave.bus.write16(address);
    slave.bus.write(Wiz::buildControlWord(block, false));
    slave.bus.read(data, len);
    slave.bus.deselect(slave.cs);
}

void Wiz5500::spiWrite(uint8_t block, uint16_t address, const uint8_t *data,
                       size_t len)
{
    // Do a manual SPI transaction
    slave.bus.configure(slave.config);

    slave.bus.select(slave.cs);
    slave.bus.write16(address);
    slave.bus.write(Wiz::buildControlWord(block, true));
    slave.bus.write(data, len);
    slave.bus.deselect(slave.cs);
}

uint8_t Wiz5500::spiRead8(uint8_t block, uint16_t address)
{
    uint8_t data;
    spiRead(block, address, &data, 1);
    return data;
}

uint16_t Wiz5500::spiRead16(uint8_t block, uint16_t address)
{
    uint16_t data;
    spiRead(block, address, reinterpret_cast<uint8_t *>(&data),
            sizeof(uint16_t));
    return fromBigEndian16(data);
}

WizIp Wiz5500::spiReadIp(uint8_t block, uint16_t address)
{
    WizIp data;
    spiRead(block, address, reinterpret_cast<uint8_t *>(&data), sizeof(WizIp));
    return data;
}

/*WizMac Wiz5500::spiReadMac(uint8_t block, uint16_t address)
{
    WizMac data;
    spiRead(block, address, reinterpret_cast<uint8_t *>(&data), sizeof(WizMac));
    return data;
}*/

void Wiz5500::spiWrite8(uint8_t block, uint16_t address, uint8_t data)
{
    spiWrite(block, address, &data, 1);
}

void Wiz5500::spiWrite16(uint8_t block, uint16_t address, uint16_t data)
{
    data = toBigEndian16(data);
    spiWrite(block, address, reinterpret_cast<uint8_t *>(&data),
             sizeof(uint16_t));
}

void Wiz5500::spiWriteIp(uint8_t block, uint16_t address, WizIp data)
{
    spiWrite(block, address, reinterpret_cast<uint8_t *>(&data), sizeof(WizIp));
}

void Wiz5500::spiWriteMac(uint8_t block, uint16_t address, WizMac data)
{
    spiWrite(block, address, reinterpret_cast<uint8_t *>(&data),
             sizeof(WizMac));
}