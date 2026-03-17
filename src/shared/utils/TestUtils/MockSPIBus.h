/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <drivers/spi/SPIBusInterface.h>
#include <kernel/sync.h>
#include <utils/Debug.h>
#include <utils/TestUtils/MockGpioPin.h>

#include <cstdint>
#include <cstdio>
#include <vector>

namespace Boardcore
{

/**
 * @brief Mock SPI Bus to be used for testing communication to a single slave:
 * data are read and written to two buffers on the memory that can then be
 * checked. Operations on the mock bus will not be successfull if the
 * configuration is not correct or the "slave" is not selected.
 *
 * Usage:
 * 1. Set the expected config (data wont be written / read if the bus current
 * configuration of the bus is different from the expected one)
 * 2. Set the data to be read from the bus (inBuf).
 * 3. Perform operations. write() will write bytes in outBuffer, read() will
 * return data from inBuf.
 * 4. Check if outBuffer contains the expected data. Check if data returned from
 * read() is as expected from inBuf.
 * 5. ???
 * 6. Profit.
 */

#ifndef USE_MOCK_PERIPHERALS
#error \
    "SpiBusInterface must be built using MockGpioPin (-DUSE_MOCK_PERIPHERALS)"
#endif

using miosix::FastMutex;
using miosix::Lock;

class MockSPIBus : public SPIBusInterface
{
public:
    MockSPIBus(SPIBusConfig expectedConfig, MockGpioPin cs)
        : expectedConfig(expectedConfig), cs(cs)
    {
    }

    ~MockSPIBus() {}

    // Delete copy/move contructors/operators
    MockSPIBus(const MockSPIBus&)            = delete;
    MockSPIBus& operator=(const MockSPIBus&) = delete;

    MockSPIBus(MockSPIBus&&)            = delete;
    MockSPIBus& operator=(MockSPIBus&&) = delete;

    /**
     * @brief Retrieve the pointer to the peripheral currently used.
     */
    SPI_TypeDef* getSpi() override { return nullptr; }

    void configure(const SPIBusConfig& newConfig) override;

    void transfer(const uint8_t* txData, uint8_t* rxData, size_t size) override;

    std::vector<uint8_t>& getOutBuf()
    {
        Lock<FastMutex> l(mutex);
        return outBuffer;
    }

    std::vector<uint8_t>& getInBuf()
    {
        Lock<FastMutex> l(mutex);
        return inBuf;
    }

    void clearBuffers()
    {
        outBuffer.clear();
        inBuf.clear();
        inBufPosCntr = 0;
    }

protected:
    FastMutex mutex;

    std::vector<uint8_t> outBuffer;  // Data written on the bus are stored here
    std::vector<uint8_t> inBuf;      // Store here data to be read from the bus
    size_t inBufPosCntr = 0;

    SPIBusConfig expectedConfig;  // Expected configuration of the bus
    MockGpioPin cs;               // Chip select pin

    uint8_t _read();

    void _write(uint8_t);

    bool canCommunicate();

    SPIBusConfig currentConfig;
};

inline bool MockSPIBus::canCommunicate()
{
    bool selected = cs.value() == 0;  // Active low
    bool result   = selected && currentConfig == expectedConfig;
    if (!result)
    {
        TRACE("Error, cannot communicato on MockSPIBus: ");
        if (!selected)
            TRACE("Chip select not asserted\n");
        else
            TRACE("Incorrect configuration\n");
    }
    return result;
}

inline void MockSPIBus::_write(uint8_t byte)
{
    if (canCommunicate())
        outBuffer.push_back(byte);
    else
        outBuffer.push_back(0);
}

inline uint8_t MockSPIBus::_read()
{
    if (canCommunicate())
    {
        if (inBufPosCntr < inBuf.size())
            return inBuf[inBufPosCntr++];
    }
    return 0;
}

inline void MockSPIBus::configure(const SPIBusConfig& config)
{
    Lock<FastMutex> l(mutex);

    this->configure(config);

    currentConfig = config;
}

inline void MockSPIBus::transfer(const uint8_t* txData, uint8_t* rxData,
                                 size_t size)
{
    Lock<FastMutex> l(mutex);

    if (txData && rxData)
    {
        for (size_t i = 0; i < size; i++)
        {
            _write(txData[i]);
            rxData[i] = _read();
        }
    }
    else if (txData)
    {
        for (size_t i = 0; i < size; i++)
            _write(txData[i]);
    }
    else if (rxData)
    {
        for (size_t i = 0; i < size; i++)
            rxData[i] = _read();
    }
    else
    {
        for (size_t i = 0; i < size; i++)
            _write(0);
    }
}

}  // namespace Boardcore
