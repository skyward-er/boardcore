/*
 * UDP communication manager
 * Copyright (c) 2017 Skyward Experimental Rocketry
 * Author: Silvano Seva
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef UDPMANAGER_H
#define UDPMANAGER_H

#include <Common.h>
#include <Singleton.h>
#include <e20/e20.h>
#include "W5200/w5200.h"
#include "PacketBuffer.h"
#include "WatchdogTimer.h"

class UdpManager : Singleton<UdpManager> {
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
    void readPacket(uint8_t *ip, uint16_t& port, void *data);
    
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
    static constexpr unsigned int TX_TIMEOUT = 500;     //500ms timeout
    
    //Size of the event queue, used for event handling
    static constexpr unsigned int EVT_QUEUE_SIZE = 20;
    
    //Size of buffer used to store packets still to be sent, IN BYTES
    static constexpr unsigned int TX_BUF_SIZE = 100000;
    
    //Size of buffer used to store packets received, IN BYTES
    static constexpr unsigned int RX_BUF_SIZE = 100000;
    
    //TX and RX socket number, used by the phy interface
    static constexpr uint8_t PHY_TX_SOCK_NUM = 0;
    static constexpr uint8_t PHY_RX_SOCK_NUM = 1;

    
    miosix::FixedEventQueue<EVT_QUEUE_SIZE> evtQueue;
    PacketBuffer *txBuffer;
    PacketBuffer *rxBuffer;
    W5200& phy = W5200::instance();    
    WatchdogTimer *wdt = Singleton< WatchdogTimer >::getInstance();
    
    //Function used by the event management thread
    friend void _evt_mgmt_thread(void *args);   
    
    void tx_handler();      //< Tx event handler function
    void tx_end_handler();  //< Tx end event handler function
    void rx_handler();      //< New packet Rx handler function
    void timeout_handler(); //< Tx timeout handler function
    
    UdpManager();
    UdpManager(const UdpManager& other);
    UdpManager& operator=(const UdpManager& other);
    ~UdpManager();
};

#endif // UDPMANAGER_H
