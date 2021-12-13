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

#include <drivers/serial_stm32.h>

#include <memory>
#include <stdexcept>

#include "../PDU.h"
#include "Common.h"

#if defined SERIAL_2_DMA
#error Serial 2 has DMA enabled, modbus RTU interface requires it to be disabled
#elif defined SERIAL_3_DMA
#error Serial 3 has DMA enabled, modbus RTU interface requires it to be disabled
#endif

/**
 * This class provides an interface for Modbus RTU slave devices. The physical
 * interface is handled by using the STM32Serial driver class, so only USART 1
 * USART 2 and USART 3 are supported.
 * NOTE: this interface currently DOESN'T SUPPORT BROADCAST messages; any
 * message of this type will be ignored.
 */

class SlaveInterface
{
public:
    /**
     * Constructor needs these parameters:
     * @param phy pointer to an STM32Serial class instance that manages the
     * serial port used for the communication
     * @param rxEn pointer to a GpioPin class that manages the pin used for
     * RS485 transceiver's RX/TX switching.
     * @param addr address assigned to the slave, must be in the range 1 - 247
     * otherwise a std::invalid_argument exception is thrown
     */
    SlaveInterface(miosix::STM32Serial *phy, miosix::gpioPin *rxEn,
                   uint8_t addr);
    ~SlaveInterface();

    /**
     * Send a reply message to a server's request
     * @param reply std::unique_ptr to a PDU class that contains the body of
     * the message
     */
    void sendReply(std::unique_ptr<PDU> reply);

    /**
     * Check if new messages for this slave are arrived. User shoul periodically
     * call this function
     */
    void receive();

    /**
     * Get the latest packet received. If there is no new packet a nullptr is
     * returned.
     * @return std::unique_ptr to a PDU class contains the message's body
     */
    std::unique_ptr<PDU> getPacket();

    // Unused functions
    SlaveInterface(const SlaveInterface &other) = delete;
    SlaveInterface &operator=(const SlaveInterface &other) = delete;
    bool operator==(const SlaveInterface &other)           = delete;

private:
    std::unique_ptr<PDU> received;  ///< last packet received
    uint8_t addr;                   ///< address of this slave
    miosix::STM32Serial *phy;
    miosix::gpioPin *rxEn;

    /**
     * This function calculates the CRC accordingly to the Modbus RTU
     * specification.
     */
    uint16_t CRC16(uint8_t *data, size_t len);
};
