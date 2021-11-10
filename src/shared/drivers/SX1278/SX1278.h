/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <drivers/spi/SPIDriver.h>

#include <cstdint>

#include "SX1278Defs.h"

/**
 * @brief Various SX1278 register/enums definitions.
 */
class SX1278
{
public:
    /**
     * @brief Requested SX1278 configuration.
     */
    struct Config
    {
    };

    SX1278(SPIBusInterface& bus, GpioPin cs, Config config = {});

    /**
     * @brief Setup the device.
     */
    void init();

    /**
     * @brief Read the chip revision.
     * 
     * @return It should always return 0x12.
     */
    uint8_t getVersion() const;

    /**
     * @brief Receive data.
     * 
     * @param buf Buffer to put the received data.
     * @return How many bytes were received.
     */
    uint8_t recv(uint8_t *buf);

    /**
     * @brief Send data.
     * 
     * @param buf Buffer with the data to send.
     * @param len Length of buffer.
     */
    void send(const uint8_t *buf, uint8_t len);

    void setBitrate(float bitrate);
    void setFreqDev(int freq_dev);
    void setFreqRF(int freq_rf);

    void debugDumpRegisters();

private:
    using Mode = SX1278Defs::RegOpMode::Mode;

    void enterMode(Mode mode);
    void waitForIrq1(uint8_t mask);
    void waitForIrq2(uint8_t mask);

    SPISlave slave;
    Config config;
    Mode mode;
};