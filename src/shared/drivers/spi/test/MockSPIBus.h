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

#include <Debug.h>

#include <cstdint>
#include <cstdio>
#include <vector>

#include "../SPIBusInterface.h"

using std::vector;

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
 * 2. Set the data to be read from the bus (in_buf).
 * 3. Perform operations. write() will write bytes in out_buf, read() will
 * return data from in_buf.
 * 4. Check if out_buf contains the expected data. Check if data returned from
 * read() is as expected from in_buf.
 * 5. ???
 * 6. Profit.
 */

#ifndef USE_MOCK_PERIPHERALS
#error \
    "SpiBusInterface must be built using MockGpioPin (-DUSE_MOCK_PERIPHERALS)"
#endif

#include <utils/testutils/MockGpioPin.h>

using miosix::FastMutex;
using miosix::Lock;
class MockSPIBus : public SPIBusInterface
{
public:
    MockSPIBus(SPIBusConfig expected_config) : expected_config(expected_config)
    {
    }

    ~MockSPIBus() {}

    // Delete copy/move contructors/operators
    MockSPIBus(const MockSPIBus&) = delete;
    MockSPIBus& operator=(const MockSPIBus&) = delete;

    MockSPIBus(MockSPIBus&&) = delete;
    MockSPIBus& operator=(MockSPIBus&&) = delete;

    /**
     * @brief See SPIBusInterface::write()
     */
    virtual void write(uint8_t byte) override;

    /**
     * @brief See SPIBusInterface::write()
     */
    virtual void write(uint8_t* data, size_t size) override;

    /**
     * @brief See SPIBusInterface::read()
     */
    virtual uint8_t read() override;

    /**
     * @brief See SPIBusInterface::read()
     */
    void read(uint8_t* data, size_t size) override;

    /**
     * @brief See SPIBusInterface::transfer()
     */
    virtual uint8_t transfer(uint8_t data) override;

    /**
     * @brief See SPIBusInterface::transfer()
     */
    virtual void transfer(uint8_t* data, size_t size) override;

    /**
     * @brief See SPIBusInterface::select()
     *
     */
    virtual void select(GpioType& cs) override;

    /**
     * @brief See SPIBusInterface::deselect()
     *
     */
    virtual void deselect(GpioType& cs) override;

    /**
     * @brief See SPIBusInterface::acquire()
     */
    virtual void acquire(SPIBusConfig config) override;

    /**
     * @brief Wether the chip select is asserted or not
     */
    virtual bool isSelected()
    {

        Lock<FastMutex> l(mutex);
        return selected;
    }

    virtual vector<uint8_t> getOutBuf()
    {
        Lock<FastMutex> l(mutex);
        return out_buf;
    }

    virtual vector<uint8_t> getInBuf()
    {
        Lock<FastMutex> l(mutex);
        return in_buf;
    }

    virtual void push(uint8_t* data, size_t len);

    virtual void clearBuffers()
    {
        out_buf.clear();
        in_buf.clear();
        in_buf_pos_cntr = 0;
    }

protected:
    FastMutex mutex;

    vector<uint8_t> out_buf;  // Data written on the bus are stored here
    vector<uint8_t> in_buf;   // Store here data to be read from the bus
    size_t in_buf_pos_cntr = 0;

    SPIBusConfig expected_config;  // Expected configuration of the bus

    virtual uint8_t _read();
    virtual void _write(uint8_t);

    virtual void _push(uint8_t* data, size_t len);

    bool canCommunicate();

    SPIBusConfig current_config;
    bool selected = false;
};

inline bool MockSPIBus::canCommunicate()
{
    bool result = selected && current_config == expected_config;
    if (!result)
    {
        TRACE("Error, cannot communicato on MockSPIBus: ");
        if (!selected)
        {
            TRACE("Chip select not asserted\n");
        }
        else
        {
            TRACE("Incorrect configuration\n");
        }
    }
    return result;
}

inline void MockSPIBus::_write(uint8_t byte)
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

inline void MockSPIBus::write(uint8_t byte)
{
    Lock<FastMutex> l(mutex);
    _write(byte);
}

inline void MockSPIBus::write(uint8_t* data, size_t size)
{
    Lock<FastMutex> l(mutex);

    for (size_t i = 0; i < size; i++)
    {
        _write(data[i]);
    }
}

inline uint8_t MockSPIBus::read()
{
    Lock<FastMutex> l(mutex);
    return _read();
}

inline void MockSPIBus::read(uint8_t* data, size_t size)
{
    Lock<FastMutex> l(mutex);
    for (size_t i = 0; i < size; i++)
    {
        *data = _read();
        data++;
    }
}

inline uint8_t MockSPIBus::_read()
{
    if (canCommunicate())
    {
        if (in_buf_pos_cntr < in_buf.size())
        {
            return in_buf[in_buf_pos_cntr++];
        }
    }
    return 0;
}

inline uint8_t MockSPIBus::transfer(uint8_t data)
{
    Lock<FastMutex> l(mutex);
    _write(data);
    return _read();
}

inline void MockSPIBus::transfer(uint8_t* data, size_t size)
{
    Lock<FastMutex> l(mutex);
    for (size_t i = 0; i < size; i++)
    {
        _write(data[i]);
        data[i] = _read();
    }
}

inline void MockSPIBus::select(GpioType& cs)
{
    Lock<FastMutex> l(mutex);
    cs.low();
    selected = true;
}

inline void MockSPIBus::deselect(GpioType& cs)
{
    Lock<FastMutex> l(mutex);
    cs.high();
    selected = false;
}

inline void MockSPIBus::acquire(SPIBusConfig config)
{
    Lock<FastMutex> l(mutex);

    SPIBusInterface::acquire(config);

    current_config = config;
}

inline void MockSPIBus::push(uint8_t* data, size_t len)
{
    Lock<FastMutex> l(mutex);
    _push(data, len);
}

inline void MockSPIBus::_push(uint8_t* data, size_t len)
{
    in_buf.insert(in_buf.end(), data, data + len);
}

}  // namespace Boardcore
