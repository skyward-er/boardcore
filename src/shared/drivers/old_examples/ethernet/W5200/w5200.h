/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
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

#include "spi_impl.h"
#include "w5200_defs.h"

typedef uint8_t SOCKET;

// extern W5200& w5200; //a W5200 driver class instance

class W5200
{
public:
    /**
     * \return a singleton instance of W5200 class
     */
    static W5200 &instance();

    /**
     * Set chip's MAC address
     */
    void setMacAddress(const uint8_t *address);

    /**
     * Set chip's IP address
     */
    void setIpAddress(const uint8_t *address);

    /**
     * Set network's subnet mask
     */
    void setSubnetMask(const uint8_t *mask);

    /**
     * Set network's gateway IP address
     */
    void setGatewayAddress(const uint8_t *address);

    /**
     * Set Mode register flags
     */
    void setModeReg(uint8_t value);

    /**
     * Set retransmission timeout period in units of 100us each
     * for example to set timeout period to 400ms you have to set
     * this register to 0x4000
     */
    void setRetryTime(uint16_t value);

    /**
     * Configures the number of retransmissions
     */
    void setRetryCount(uint16_t value);

    /**
     * Configures the interrupt mask for socket interrupts
     */
    void setInterruptMask(uint8_t mask);

    /**
     * \return chip's interrupt register value
     */
    uint8_t readInterruptReg() const;

    /**
     * \return chip's IR2 register value, please refer to datasheet
     */
    uint8_t readSocketInterruptReg() const;

    /**
     * Configures chip's IMR2 register, please refer to datasheet
     */
    void setSocketInterruptMask(uint8_t mask);

    /**
     * \return value of register that indicates chip's
     * physical status
     */
    uint8_t getPhyStatus() const;

    /**
     * Writes socket's mode register
     * \param sockNum: socket number, between 0 and 7
     * \param value: value to be written
     */
    void setSocketModeReg(SOCKET sockNum, uint8_t value);

    /**
     * Used to send a command to a socket through its command register
     * \param sockNum: socket number, between 0 and 7
     * \param value: command opcode
     */
    void setSocketCommandReg(SOCKET sockNum, uint8_t value);

    /**
     * Used to get register's value, as a way to check if a given
     * command is completed
     * \param sockNum: socket number, between 0 and 7
     * \return register's value
     */
    uint8_t getSocketCommandReg(SOCKET sockNum) const;

    /**
     * Configures the socket interrupts that will be signalled
     * \param sockNum: socket number, between 0 and 7
     * \param value: mask value
     */
    void setSocketInterruptMaskReg(SOCKET sockNum, uint8_t value);

    /**
     * Reads socket's interrupt register
     * \param sockNum: socket number, between 0 and 7
     * \return socket's interrupt register value
     */
    uint8_t getSocketInterruptReg(SOCKET sockNum) const;

    /**
     * Resets all the socket's interrupt register flags
     * \param sockNum: socket number, between 0 and 7
     */

    void clearSocketInterruptReg(SOCKET sockNum);

    /**
     * Reads socket's status register
     * \param sockNum: socket number, between 0 and 7
     * \return socket's status register value
     */
    uint8_t getSocketStatusReg(SOCKET sockNum) const;

    /**
     * Sets socket's source port value, both for TCP and UDP
     * \param sockNum: socket number, between 0 and 7
     * \param port: socket's source port number
     */
    void setSocketSourcePort(SOCKET sockNum, uint16_t port);

    /**
     * Sets socket's destination MAC address
     * \param sockNum: socket number, between 0 and 7
     * \param destMAC: socket's destination MAC address
     */
    void setSocketDestMac(SOCKET sockNum, uint8_t *destMAC);

    /**
     * Sets socket's destination IP address
     * \param sockNum: socket number, between 0 and 7
     * \param destIP: socket's destination IP address
     */
    void setSocketDestIp(SOCKET sockNum, const uint8_t *destIP);

    /**
     * Sets socket's destination port number
     * \param sockNum: socket number, between 0 and 7
     * \param value: socket's destination port number
     */
    void setSocketDestPort(SOCKET sockNum, uint16_t destPort);

    /**
     * Set socket's Maximum Segment Size for TCP mode
     * \param sockNum: socket number, between 0 and 7
     * \param value: MSS value
     */
    void setSocketMSS(SOCKET sockNum, uint16_t value);

    /**
     * Set socket's protocol number when used in IPraw mode
     * \param sockNum: socket number, between 0 and 7
     * \param value: protocol number
     */
    void setSocketProtocolValue(SOCKET sockNum, uint8_t value);

    /**    W5200();
     * Sets Type Of Service field value in socket's IP header
     * \param sockNum: socket number, between 0 and 7
     * \param TOSvalue: field value
     */
    void setSocketTos(SOCKET sockNum, uint8_t TOSvalue);

    /**
     * Sets Time To Live field value in socket's IP header
     * \param sockNum: socket number, between 0 and 7
     * \param TTLvalue: field value
     */
    void setSocketTtl(SOCKET sockNum, uint8_t TTLvalue);

    /**
     * Sets Fragment field value in socket's IP header
     * \param sockNum: socket number, between 0 and 7
     * \param value: field value
     */
    void setSocketFragmentValue(SOCKET sockNum, uint16_t value);

    /**
     * Configures socket's internal RX memory size
     * Accepted values are: 0, 1, 2, 4, 8 and 16kB
     * \param sockNum: socket number, between 0 and 7
     * \param memSize: desired RX memory size
     */
    void setSocketRxMemSize(SOCKET sockNum, uint8_t memSize);

    /**
     * Configures socket's internal TX memory size
     * Accepted values are: 0, 1, 2, 4, 8 and 16kB
     * \param sockNum: socket number, between 0 and 7
     * \param memSize: desired TX memory size
     */
    void setSocketTxMemSize(SOCKET sockNum, uint8_t memSize);

    /**
     * Writes data into socket TX buffer and updates in-chip pointer
     * \param sockNum: socket number, between 0 and 7
     * \param data: pointer to buffer containing data to be written
     * \param len: number of bytes to be written
     */
    void writeData(SOCKET sockNum, const uint8_t *data, uint16_t len);

    /**
     * Reads data from socket RX buffer and updates in-chip pointer
     * \param sockNum: socket number, between 0 and 7
     * \param data: pointer to buffer in which write data
     * \param len: number of bytes to be read
     */
    void readData(SOCKET sockNum, uint8_t *data, uint16_t len);

    /**
     * \param sockNum: socket number, between 0 and 7
     * \return the received data size in byte
     */
    uint16_t getReceivedSize(SOCKET sockNum) const;

private:
    W5200();

    /**
     * Write one byte into chip's register
     * \param address: register's address
     * \param data: data to be written
     */
    void writeRegister(uint16_t address, uint8_t data);

    /**
     * Write multiple bytes into chip's memory
     * \param address: writing process start point address
     * \param data: pointer to the data to be written
     * \param len: number of bytes to be written
     */
    void writeBuffer(uint16_t address, const uint8_t *data, uint16_t len);

    /**
     * Read one byte from chip's register
     * \param address: register's address
     * \return data read
     */
    uint8_t readRegister(uint16_t address) const;

    /**
     * Read multiple bytes into chip's memory
     * \param address: reading process start point address
     * \param data: pointer to the data to be read
     * \param len: number of bytes to be read
     */
    void readBuffer(uint16_t address, uint8_t *data, uint16_t len);

    /**
     * This function is used to copy data from application buffer to socket's
     * in-chip TX buffer
     *
     * TODO: better description
     *
     * \param socket: socket number, between 0 and 7
     * \param src: pointer to source buffer
     * \param dst: destination buffer start address.
     *             This start address is referred to chip's buffer!!
     * \param len: number of bytes to be copied
     */
    void writeTxBuf(SOCKET socket, const uint8_t *src, uint16_t dst,
                    uint16_t len);

    /**
     * This function is used to copy data from socket's in-chip TX buffer
     * to application buffer
     *
     * TODO: better description
     *
     * \param socket: socket number, between 0 and 7
     * \param src: source buffer start address.
     *             This start address is referred to chip's buffer!!
     * \param dst: pointer to destination buffer
     * \param len: number of bytes to be copied
     */
    void readRxBuf(SOCKET socket, uint16_t src, uint8_t *dst, uint16_t len);

    uint16_t txBufSize[MAX_SOCK_NUM];  // sockets TX buffer size in byte
    uint16_t rxBufSize[MAX_SOCK_NUM];  // sockets RX buffer size in byte

    const uint8_t macAddress[6] = {0xde, 0xad, 0x00, 0x00, 0xbe, 0xef};
};
