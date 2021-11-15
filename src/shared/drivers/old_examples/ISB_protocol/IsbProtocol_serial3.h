/* Copyright (c) 2017 Skyward Experimental Rocketry
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

#include <cstdio>
#include <cstdint>
#include <stddef.h>
#include <cstring>
#include "miosix.h"

/* This class is a kind of driver to be used to excange data between the 
 * stormtrooper master and the other stormtrooper slave boards through the 
 * RS485 lines. The communication model is this: all the boards are connected
 * on the same bus and each board has a unique node address. To avoid conflicts,
 * the master is the only device on the bus that can initiate a communication;
 * thus, to communicate with a specific slave, the master sends a packet 
 * containing the targeted slave address and the data and then waits for the 
 * response (if expected). Owing to this model, if two stormtrooper slaves need
 * to exchange data they have to rely on the stormtrooper master acting as a
 * router between them.
 *
 * The packet structure is this:
 * 
 * +------+-----+------+-----+
 * | addr | len | data | CRC |
 * +------+-----+------+-----+
 * 
 * -> addr: target node address, 8 bit
 * -> len: lenght of the data field, 8 bit
 * -> data: data bytes, up to 255
 * -> CRC: 16 bit field calculated on addr, len and data using the ccitt CRC16
 *         formula
 * 
 * On the bus data is sent using 9 bit long frames with one stop bit. If the 9th
 * bit is set to 1 means that the byte received is an address byte and so a 
 * packet is incoming
 * 
 * 
 * Here is a sample code about this class' usage:
 * 
 * auto comm = IsbProtocol_serial2::instance();
 *
 * comm.setBaud(9600);           //set baud to 9600 bps 
 * comm.setNodeAddress(0xAB);    //set node ID
 *
 * // this to check if new data arrived and to copy the bytes into a local buffer
 * 
 * uint8_t rx_buf[256];
 * 
 * if(comm.newDataAvailable() > 0)
 * {
 *     comm.getData(rx_buf);
 * }
 * 
 * // this to send data
 * 
 * uint8_t tx_buf[] = "Some data for you! :)";
 * 
 * comm.sendData(0xE0, tx_buf, sizeof(tx_buf));
 * 
 */

class IsbProtocol_serial3
{
public:
    
    /**
     * @return singleton intance of the class
     */
    static IsbProtocol_serial3& instance();
    
    ~IsbProtocol_serial3();
    
    /**
     * Set the USART baud rate.
     * @param baud baud rate to be set
     */
    void setBaud(uint32_t baud);
    
    /**
     * Set the node's address in the network
     */
    void setNodeAddress(uint8_t address);
    
    /**
     * @return number of payload data bytes received or 0 if none received
     */
    size_t newDataAvailable();
    
    /**
     * Get the latest payload data received.
     * @param buffer pointer to buffer in which copy the new data, it must have
     * dimension greater than or equal to the value returned by newDataAvailable()
     */
    void getData(uint8_t *buffer);
    
    /**
     * Send a packet, this function blocks until all data is sent
     * 
     * @param dstAddr destination node address
     * @param data pointer to a buffer containing the payload data
     * @param len payload size in bytes
     */
    void sendData(uint8_t dstAddr, uint8_t *data, size_t len);
    
    /**
     * Interrupt handler. DON'T CALL IT ANYWHERE!!
     */
    void IRQHandler();
    
private:
    
    uint8_t rxBuf[300];
    struct R 
    {
        uint16_t packetSize;        
        uint16_t rxIndex;
        bool rxInProgress;
        bool dataLenPending;
    } rxStatus;
    
    uint8_t txBuf[300];
    uint8_t nodeAddress;
    
    uint16_t CRC16(uint8_t *data, size_t len);
    
    IsbProtocol_serial3();
    IsbProtocol_serial3(const IsbProtocol_serial3& other);
    IsbProtocol_serial3& operator=(const IsbProtocol_serial3& other);
    bool operator==(const IsbProtocol_serial3& other);
};
