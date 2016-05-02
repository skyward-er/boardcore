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
  
    /**
     * Ctor, creates a new UDP soket which listening port is 
     * \param sockPort.
     * NOTE: <b>You can create eight socket maximum!<b> This because
     * the ethernet chip supports at least eight concurrent sockets
     */
    UdpSocket(const uint16_t& sockPort);
    
    ~UdpSocket();
    
    /**
     * Send data to a host. This is a locking function, calling thread
     * is put to sleep until sending process terminates.
     * \param destIp remote host's IP address
     * \param destPort remote host's listening port
     * \param data pointer to buffer of data to be sent
     * \param len number of bytes to be sent
     * \return true if sending was ok, false on failure
     */
    bool sendTo(const uint8_t *destIp, const uint16_t& destPort, const uint8_t *data, uint16_t& len);
    
    /**
     * Listen for incoming data. This is a locking function, calling thread
     * is put to sleep until some data is received.
     * \param sourceIp pointer to buffer in which store remote host's IP address
     * \param soucePort pointer to a variable in which store remote host's source port
     * \param data pointer to buffer in which store the data received
     * \return number of bytes received
     */
    uint16_t receive(uint8_t *sourceIp, uint16_t *sourcePort, uint8_t *data);
    
    /**
     * IRQ handler function used internally.
     * <b> NEVER CALL THIS!!! <b>
     */
    static void irqHandler();   

private:    
    UdpSocket& operator=(const UdpSocket& other);
    bool operator==(const UdpSocket& other);
        
    /**
     * Internally used, it initializes chip and MCU's interrupt registers
     */
    void init();
    
    static const uint8_t maxSockNum = 8;
    static uint8_t instanceIndex;           //used to keep track of which chip's sockets are used
    static bool initialized;                //flag used to check if chip was initialized or not
    uint8_t sockn;        
    static uint8_t interruptFlags[8];       //content of socket's interrupt register
    static W5200& w5200;
};

#endif // UDPSOCKET_H
