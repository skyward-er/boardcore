/*
 *
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

class UdpManager : Singleton<UdpManager> {
    friend class Singleton<UdpManager>;
public:
//     static UdpManager& instance();
    void setPort(uint16_t port);
    void sendPacketTo(const uint8_t* ip, const uint16_t port, const void* data, 
                                                                 size_t len);
    bool newReceivedPackets();
    void readPacket();
    
    //Internally used handler!!
    void IRQ_handler(uint8_t irqNum);
    
    static enum  {
        
        IRQ_PHY = 0,    //IRQ from ethernet chip
        IRQ_TMO = 1     //IRQ from timeout timer
    }IRQ_source;
    
private:
    
    static const unsigned int EVT_QUEUE_SIZE = 20;
    static const unsigned int TX_BUF_SIZE = 100000;
    static const unsigned int RX_BUF_SIZE = 100000;
    static const uint8_t PHY_TX_SOCK_NUM = 0;
    static const uint8_t PHY_RX_SOCK_NUM = 1;

    
    W5200& phy = W5200::instance();
    uint16_t rxPacketCounter;
    
    miosix::FixedEventQueue<EVT_QUEUE_SIZE> evtQueue;
    
    PacketBuffer *txBuffer;
    PacketBuffer *rxBuffer;
    
    friend void _evt_mgmt_thread(void *args);   
    
    void tx_handler();
    void tx_end_handler();
    void rx_handler();
    void timeout_handler();   
    
    UdpManager();
    UdpManager(const UdpManager& other);
    UdpManager& operator=(const UdpManager& other);
    ~UdpManager();
};

#define udpComm Singleton<UdpManager>::getInstance()

#endif // UDPMANAGER_H
