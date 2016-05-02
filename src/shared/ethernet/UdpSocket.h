/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (c) 2016 Skyward Experimental Rocketry
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

#ifndef UDPSOCKET_H
#define UDPSOCKET_H

#include "W5200/w5200.h"
#include <miosix.h>
#include <kernel/scheduler/scheduler.h>
#include <stdexcept>
#include <string>

class UdpSocket
{
public:
    UdpSocket(const uint16_t& sockPort);
    ~UdpSocket();
    bool sendTo(const uint8_t *destIp, const uint16_t& destPort, const uint8_t *data, uint16_t& len);
    uint16_t receive(uint8_t *sourceIp, uint16_t *sourcePort, uint8_t *data);
    
    static void irqHandler();   

private:    
    UdpSocket& operator=(const UdpSocket& other);
    bool operator==(const UdpSocket& other);
        
    void init();
    
    static const uint8_t maxSockNum = 8;
    static uint8_t instanceIndex;           //used to keep track of which chip's sockets are used
    static bool initialized;                //flag used to check if chip was initialized or not
    uint8_t sockn;        
    static uint8_t interruptFlags[8];       //content of socket's interrupt register
    static W5200& w5200;
};

#endif // UDPSOCKET_H
