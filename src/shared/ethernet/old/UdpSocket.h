/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Silvano Seva
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef UDPSOCKET_H
#define UDPSOCKET_H

#include <Common.h>
#include "W5200/w5200.h"

class UdpManager
{
public:
    /**
     * Ctor, creates a new UDP soket which listening port is 
     * \param sockPort.
     * NOTE: <b>You can create eight socket maximum!<b> This because
     * the ethernet chip supports at least eight concurrent sockets
     */
    UdpManager(const uint16_t& sockPort);
    
    ~UdpManager();
    
    /**
     * Send data to a host. This is a locking function, calling thread
     * is put to sleep until sending process terminates.
     * \param destIp remote host's IP address
     * \param destPort remote host's listening port
     * \param data pointer to buffer of data to be sent
     * \param len number of bytes to be sent
     * \return true if sending was ok, false on failure
     */
    bool sendTo(const uint8_t *destIp, const uint16_t destPort, 
                const void *data, uint16_t len);
    
    /**
     * Listen for incoming data. This is a locking function, calling thread
     * is put to sleep until some data is received.
     * \param sourceIp pointer to buffer in which store remote host's IP address
     * \param soucePort pointer to a variable in which store remote host's 
     *                  source port
     * \param data pointer to buffer in which store the data received
     * \return number of bytes received
     */
    uint16_t receive(uint8_t *sourceIp, uint16_t *sourcePort, void *data);
    
    /**
     * IRQ handler function used internally.
     * <b> NEVER CALL THIS MANUALLY!!! <b>
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
    static uint8_t instanceIndex;     // Used to keep track of which chip's 
                                      // sockets are used
    static bool initialized;          // Flag used to check if chip was 
                                      // initialized or not
    uint8_t sockn;        
    static uint8_t interruptFlags[8]; // Content of socket's interrupt register
    static W5200& w5200;
};

#endif // UDPSOCKET_H
