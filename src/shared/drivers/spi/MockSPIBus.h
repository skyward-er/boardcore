/**
 * Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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

#pragma once

#include <cstdint>
#include <vector>

#include "SPIInterface.h"

using std::vector;

/**
 * @brief Mock SPI Bus to be used for testing: data are read and written to two
 * buffers on the memory can then be checked.
 *
 * Usage:
 * 1. Set the expected config (data wont be written / read if the bus current
 * configuration of the bus is different from the expected one)
 * 2. Set the data to be read from the bus (in_buf).
 * 3. Perform operations. write() will write bytes in out_buf, read() will
 * return data from in_buf.
 * 4. Check if out_buf contains the expected data. Check if data returned from
 * read() is as expected from in_buf.
 * 5. ???
 * 6. Profit.
 */
class MockSPIBus : public SPIBusInterface
{
public:
    MockSPIBus(){}
    ~MockSPIBus() {}

    // Delete copy/move contructors/operators
    MockSPIBus(const MockSPIBus&) = delete;
    MockSPIBus& operator=(const MockSPIBus&) = delete;

    MockSPIBus(MockSPIBus&&) = delete;
    MockSPIBus& operator=(MockSPIBus&&) = delete;

    /**
     * @brief See SPIBusInterface::write()
     */
    void write(uint8_t byte) override;

    /**
     * @brief See SPIBusInterface::write()
     */
    void write(uint8_t* data, size_t size) override;

    /**
     * @brief See SPIBusInterface::read()
     */
    uint8_t read() override;

    /**
     * @brief See SPIBusInterface::read()
     */
    void read(uint8_t* data, size_t size) override;

    /**
     * @brief See SPIBusInterface::transfer()
     */
    uint8_t transfer(uint8_t data) override;

    /**
     * @brief See SPIBusInterface::transfer()
     */
    void transfer(uint8_t* data, size_t size) override;

    /**
     * @brief See SPIBusInterface::select()
     */
    void select(GpioPin& cs) override;

    /**
     * @brief See SPIBusInterface::deselect()
     */
    void deselect(GpioPin& cs) override;

    /**
     * @brief See SPIBusInterface::configure()
     */
    void configure(SPIBusConfig config) override;

    vector<uint8_t> out_buf;  // Data written on the bus are stored here
    vector<uint8_t> in_buf;   // Store here data to be read from the bus

    unsigned int in_buf_pos = 0;  // Read data iterator

    SPIBusConfig expected_config;  // Expected configuration of the bus

private:
    bool canCommunicate();

    SPIBusConfig current_config;
    bool selected = false;
};

bool MockSPIBus::canCommunicate()
{
    return selected && current_config == expected_config;
}

void MockSPIBus::write(uint8_t byte)
{
    if (canCommunicate())
    {
        out_buf.push_back(byte);
    }
    else
    {
        out_buf.push_back(0);
    }
}

void MockSPIBus::write(uint8_t* data, size_t size)
{
    if (canCommunicate())
    {
        out_buf.insert(out_buf.end(), data, data + size);
    }
    else
    {
        out_buf.insert(out_buf.end(), size, 0);
    }
}

uint8_t MockSPIBus::read()
{
    if (canCommunicate())
    {
        return in_buf[in_buf_pos++];
    }
    return 0;
}

void MockSPIBus::read(uint8_t* data, size_t size)
{
    if (canCommunicate())
    {
        for (size_t i = 0; i < size; i++)
        {
            *data = in_buf[in_buf_pos++];
            data++;
        }
    }
    else
    {
        for (size_t i = 0; i < size; i++)
        {
            *data = 0;
            data++;
        }
    }
}

uint8_t MockSPIBus::transfer(uint8_t data)
{
    if (canCommunicate())
    {

        out_buf.push_back(data);
        return in_buf[in_buf_pos++];
    }
    else
    {

        out_buf.push_back(0);
        return 0;
    }
}

void MockSPIBus::transfer(uint8_t* data, size_t size)
{
    if (canCommunicate())
    {
        for (size_t i = 0; i < size; i++)
        {
            out_buf.push_back(*data);
            *data = in_buf[in_buf_pos++];
            data++;
        }
    }
    else
    {
        for (size_t i = 0; i < size; i++)
        {
            out_buf.push_back(0);
            *data = 0;
            data++;
        }
    }
}

void MockSPIBus::select(GpioPin& cs) { selected = true; }

void MockSPIBus::deselect(GpioPin& cs) { selected = false; }

void MockSPIBus::configure(SPIBusConfig config) { current_config = config; }