/* Copyright (c) 2016-2017 Skyward Experimental Rocketry
 * Author: Silvano Seva
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
#include "UdpManager.h"

using namespace std;
using namespace miosix;

void __attribute__((naked)) EXTI1_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z13EXTIrqHandlerv");
    restoreContext();
}

void __attribute__((used)) EXTIrqHandler()
{
    EXTI->PR |= EXTI_PR_PR1;
    Singleton<UdpManager>::getInstance()->phyIrqHandler();
}

void _evt_mgmt_thread(void* args)
{
    Singleton<UdpManager>::getInstance()->evtQueue.run();
}

UdpManager::UdpManager()
{
    eth::int1::mode(Mode::INPUT);

    // Configure STM32 to generate an interrupt on
    // falling edge of chip's INT line

    {
        FastInterruptDisableLock dLock;
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
        RCC_SYNC();
    }

    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PC;
    EXTI->IMR |= EXTI_IMR_MR1;
    EXTI->FTSR |= EXTI_FTSR_TR1;

    NVIC_SetPriority(EXTI1_IRQn, 10);
    NVIC_ClearPendingIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);

    phy.setModeReg(0x00);
    phy.setSocketInterruptMask(0xFF);

    txBuffer = new PacketBuffer(TX_BUF_SIZE);
    rxBuffer = new PacketBuffer(RX_BUF_SIZE);

    wdt->setDuration(TX_TIMEOUT);
    wdt->setCallback(bind(&UdpManager::wdtIrqHandler, this));

    if (!txBuffer->isValid() || !rxBuffer->isValid())
    {
        // TODO log!
    }
    else
    {
        Thread::create(_evt_mgmt_thread, 1024);
    }
}

UdpManager::~UdpManager()
{
    delete txBuffer;
    delete rxBuffer;
}

void UdpManager::setTxPort(uint16_t port)
{
    // Open transmitting socket
    phy.setSocketModeReg(PHY_TX_SOCK_NUM, SOCKn_MR_UDP);
    phy.setSocketSourcePort(PHY_TX_SOCK_NUM, port);
    // Enable timeout & send ok interrupts
    phy.setSocketInterruptMaskReg(PHY_TX_SOCK_NUM, 0x18);
    phy.setSocketCommandReg(PHY_TX_SOCK_NUM, SOCKn_CR_OPEN);
}

void UdpManager::setRxPort(uint16_t port)
{
    // Open receiving socket
    phy.setSocketModeReg(PHY_RX_SOCK_NUM, SOCKn_MR_UDP);
    phy.setSocketSourcePort(PHY_RX_SOCK_NUM, port);
    // Enable recv interrupt only
    phy.setSocketInterruptMaskReg(PHY_RX_SOCK_NUM, 0x04);
    phy.setSocketCommandReg(PHY_RX_SOCK_NUM, SOCKn_CR_OPEN);
    phy.setSocketCommandReg(PHY_RX_SOCK_NUM, SOCKn_CR_RECV);
}

/** Legacy code, uncomment if we want tx @ port and rx @ port+1 **/

// void UdpManager::setPort(uint16_t port) {
//
//     //Open transmitting socket
//     phy.setSocketModeReg(PHY_TX_SOCK_NUM,SOCKn_MR_UDP);
//     phy.setSocketSourcePort(PHY_TX_SOCK_NUM,port);
//     // Enable timeout & send ok interrupts
//     phy.setSocketInterruptMaskReg(PHY_TX_SOCK_NUM,0x18);
//     phy.setSocketCommandReg(PHY_TX_SOCK_NUM,SOCKn_CR_OPEN);
//
//     //Open receiving socket
//     phy.setSocketModeReg(PHY_RX_SOCK_NUM,SOCKn_MR_UDP);
//     phy.setSocketSourcePort(PHY_RX_SOCK_NUM,port+1);
//     // Enable recv interrupt only
//     phy.setSocketInterruptMaskReg(PHY_RX_SOCK_NUM,0x04);
//     phy.setSocketCommandReg(PHY_RX_SOCK_NUM,SOCKn_CR_OPEN);
//     phy.setSocketCommandReg(PHY_RX_SOCK_NUM,SOCKn_CR_RECV);
// }

bool UdpManager::newReceivedPackets() { return !rxBuffer->empty(); }

void UdpManager::sendPacketTo(const uint8_t* ip, const uint16_t port,
                              const void* data, size_t len)
{
    packet_header_t header;
    header.ipAddress   = *(reinterpret_cast<const uint32_t*>(ip));
    header.port        = port;
    header.payloadSize = len;

    bool wasEmpty = txBuffer->empty();
    bool ok = txBuffer->push(header, reinterpret_cast<const uint8_t*>(data));

    if (!ok)
    {
        // TODO: better failure logging
        puts("UDP->sendPacketTo: failed to enqueue the new packet\n");
    }
    else if (wasEmpty)
    {
        bool pok =
            evtQueue.postNonBlocking(bind(&UdpManager::tx_handler, this));

        if (!pok)
        {
            // TODO: better failure logging
            puts(
                "UDP->sendPacketTo: job queue full."
                "Failed to post tx_handler.\n");
        }
    }
}

size_t UdpManager::recvPacketSize()
{
    if (rxBuffer->empty())
        return 0;

    packet_header_t hdr = rxBuffer->getHeader();
    return hdr.payloadSize;
}

void UdpManager::readPacket(uint8_t* ip, uint16_t& port, void* data)
{
    packet_header_t header = rxBuffer->getHeader();
    // IP address is 4 bytes long
    memcpy(ip, reinterpret_cast<uint8_t*>(&(header.ipAddress)), 4);
    port = header.port;

    // The copy of the exact number of bytes is guaranteed by getData. We only
    // have to be sure that the receiving buffer has the right capacity,
    // which is given by UdpManager::recvPacketSize()
    rxBuffer->getData(reinterpret_cast<uint8_t*>(data));
    rxBuffer->popFront();
}

/** Interrupt and event handlers **/
void UdpManager::phyIrqHandler()
{
    bool hppw = false;

    uint8_t sockInt = phy.readSocketInterruptReg();

    // TX socket interrupts management
    if (sockInt & (0x01 << PHY_TX_SOCK_NUM))
    {
        // Stopping watchdog inside an IRQ is safe to do
        wdt->stop();

        uint8_t txFlags = phy.getSocketInterruptReg(PHY_TX_SOCK_NUM);
        phy.clearSocketInterruptReg(PHY_TX_SOCK_NUM);

        // Send OK interrupt flag set
        if (txFlags & 0x10)
            evtQueue.IRQpost(bind(&UdpManager::tx_end_handler, this), hppw);

        // Timeout flag set, problems with ARP
        if (txFlags & 0x08)
            evtQueue.IRQpost(bind(&UdpManager::timeout_handler, this), hppw);
    }

    // RX socket interrupts management
    if (sockInt & (0x01 << PHY_RX_SOCK_NUM))
    {
        uint8_t rxFlags = phy.getSocketInterruptReg(PHY_RX_SOCK_NUM);
        phy.clearSocketInterruptReg(PHY_RX_SOCK_NUM);

        if (rxFlags & 0x04)
            evtQueue.IRQpost(bind(&UdpManager::rx_handler, this), hppw);
    }

    if (hppw)
        Scheduler::IRQfindNextThread();
}

void UdpManager::wdtIrqHandler()
{
    bool hppw = false;
    evtQueue.IRQpost(bind(&UdpManager::timeout_handler, this), hppw);

    if (hppw)
        Scheduler::IRQfindNextThread();
}

void UdpManager::tx_handler()
{
    if (txBuffer->empty())
        return;

    packet_header_t header = txBuffer->getHeader();
    uint8_t* addr          = reinterpret_cast<uint8_t*>(&header.ipAddress);

    uint8_t payload[header.payloadSize];
    txBuffer->getData(payload);
    txBuffer->popFront();

    phy.setSocketDestIp(PHY_TX_SOCK_NUM, addr);
    phy.setSocketDestPort(PHY_TX_SOCK_NUM, header.port);
    phy.writeData(PHY_TX_SOCK_NUM, payload, header.payloadSize);
    phy.setSocketCommandReg(PHY_TX_SOCK_NUM, SOCKn_CR_SEND);

    wdt->clear();
    wdt->start();
}

void UdpManager::tx_end_handler()
{
    if (txBuffer->empty())
        return;

    bool ok = evtQueue.postNonBlocking(bind(&UdpManager::tx_handler, this));
    if (!ok)
    {
        // TODO: better failure logging
        puts("UDP->tx_end_handler:job queue full, failed to post tx_handler");
    }
}

void UdpManager::timeout_handler()
{
    if (wdt->expired())
        puts("Tx timeout due to watchdog expiration");
    else
        puts("Tx timeout due to phy error");

    bool ok = evtQueue.postNonBlocking(bind(&UdpManager::tx_end_handler, this));
    if (!ok)
        puts(
            "UDP->timeout_handler: job queue full, "
            "failed to post tx_end_handler");
}

void UdpManager::rx_handler()
{
    // get new packet len, in bytes
    uint16_t len = phy.getReceivedSize(PHY_RX_SOCK_NUM);
    uint8_t buffer[len];

    // read all the packet, that is made of so30urceIp + sourcePort +
    // payload len + payload. Since the header length is of 8 bytes,
    // the payload length is len - 8 bytes! :-)
    phy.readData(PHY_RX_SOCK_NUM, buffer, len);

    // Set back to listening mode
    phy.setSocketCommandReg(PHY_RX_SOCK_NUM, SOCKn_CR_RECV);

    packet_header_t header;

    memcpy(reinterpret_cast<uint8_t*>(&(header.ipAddress)), buffer, 4);
    header.port        = (buffer[4] << 8) | buffer[5];
    header.payloadSize = len - 8;

    bool pushOk = rxBuffer->push(header, &buffer[8]);

    if (!pushOk)
        puts(
            "rx_handler error: rxBuffer is full,"
            "cannot enqueue a new packet!");
}
