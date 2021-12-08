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

#pragma once

#include <Common.h>
#include <Singleton.h>
#include <W5200/w5200.h>
#include <e20/e20.h>

#include "PacketBuffer.h"
#include "WatchdogTimer.h"

class UdpManager : Singleton<UdpManager>
{
    friend class Singleton<UdpManager>;

public:
    /**
     * Open a transmitting UDP socket which source port is @param port
     */
    void setTxPort(uint16_t port);

    /**
     * Open a receiving UDP socket that will listen on @param port, listening
     * starts as soon as this function is called
     */
    void setRxPort(uint16_t port);

    /**
     * Send an UDP packet.
     * @param ip: destination IP address
     * @param port: destination port
     * @param data: pointer to payload data
     * @param len: payload lenght in bytes
     */
    void sendPacketTo(const uint8_t* ip, const uint16_t port, const void* data,
                      size_t len);

    /**
     * @return true if there are new packets in the rx buffer
     */
    bool newReceivedPackets();

    /**
     * @return payload size of the packet stored at the head of the internal
     * buffer, in other words the packet that will be read if readPacket() is
     * called.
     */
    size_t recvPacketSize();

    /**
     * Read the packet placed at rx buffer's head popping it out of the queue.
     * @param ip: source ip address
     * @param port: source port
     * @param data: pointer to a buffer in which store the payload data. It must
     * have a size greater than or equal to that returned by recvPacketSize()
     */
    void readPacket(uint8_t* ip, uint16_t& port, void* data);

    /**
     * IRQ handler for interrupts coming from the phy interface chip
     * WARNING: DON'T call this!!
     */
    void phyIrqHandler();

    /**
     * IRQ handler for interrupts coming from the tx watchdog timer
     * WARNING: DON'T call this!!
     */
    void wdtIrqHandler();

private:
    // TX timeout time, in ms
    static constexpr unsigned int TX_TIMEOUT = 500;  // 500ms timeout

    // Size of the event queue, used for event handling
    static constexpr unsigned int EVT_QUEUE_SIZE = 20;

    // Size of buffer used to store packets still to be sent, IN BYTES
    static constexpr unsigned int TX_BUF_SIZE = 100000;

    // Size of buffer used to store packets received, IN BYTES
    static constexpr unsigned int RX_BUF_SIZE = 100000;

    // TX and RX socket number, used by the phy interface
    static constexpr uint8_t PHY_TX_SOCK_NUM = 0;
    static constexpr uint8_t PHY_RX_SOCK_NUM = 1;

    miosix::FixedEventQueue<EVT_QUEUE_SIZE> evtQueue;
    PacketBuffer* txBuffer;
    PacketBuffer* rxBuffer;
    W5200& phy         = W5200::instance();
    WatchdogTimer* wdt = Singleton<WatchdogTimer>::getInstance();

    // Function used by the event management thread
    friend void _evt_mgmt_thread(void* args);

    void tx_handler();       //< Tx event handler function
    void tx_end_handler();   //< Tx end event handler function
    void rx_handler();       //< New packet Rx handler function
    void timeout_handler();  //< Tx timeout handler function

    UdpManager();
    UdpManager(const UdpManager& other);
    UdpManager& operator=(const UdpManager& other);
    ~UdpManager();
};
