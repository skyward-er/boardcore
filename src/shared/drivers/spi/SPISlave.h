/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include "SPIBusInterface.h"

namespace Boardcore
{

/**
 * @brief Contains information about a single SPI slave device.
 */
struct SPISlave
{
private:
    struct SlaveSelect
    {
        enum class Type
        {
            CS,
            MUX
        } type;

        union
        {
            // Type::CS
            GpioType cs;

            // Type::MUX
            struct
            {
                GpioType s0;
                GpioType s1;
                GpioType s2;
                uint8_t value;
            } mux;
        } data;

        /**
         * @brief Creates a multiplexer slave select object.
         */
        static SlaveSelect MuxSelect(GpioType s0, GpioType s1, GpioType s2,
                                     uint8_t value);

        /**
         * @brief Creates a single pin slave select object.
         */
        static SlaveSelect PinSelect(GpioType cs);

        void select();

        void deselect();
    };

public:
    SPIBusInterface& bus;  ///< Bus on which the slave is connected.
    SPIBusConfig config;   ///< How the bus should be configured to
                           ///< communicate with the slave.
    SlaveSelect cs;        ///< Chip select of the slave.

    SPISlave(SPIBusInterface& bus, GpioType cs, SPIBusConfig config = {})
        : bus(bus), config(config), cs(SlaveSelect::PinSelect(cs))
    {
    }

    SPISlave(SPIBusInterface& bus, GpioType s0, GpioType s1, GpioType s2,
             uint8_t value, SPIBusConfig config = {})
        : bus(bus), config(config),
          cs(SlaveSelect::MuxSelect(s0, s1, s2, value))
    {
    }

    void configureBus() { bus.configure(config); }

    void select();

    void deselect();
};

/**
 * @brief RAII Interface for SPI chip selection.
 */
class SPISelectLock
{
public:
    explicit SPISelectLock(SPISlave& slave) : slave(slave) { slave.select(); }

    ~SPISelectLock() { slave.deselect(); }

private:
    SPISlave& slave;
};

/**
 * @brief RAII Interface for SPI bus configuration and slave selection.
 *
 * Bus configuration must happen before selection to ensure that the bus is
 * ready for communication. Here, configuration is performed in the
 * initializer list by taking advantage of the comma operator.
 */
class SPIAcquireLock
{
public:
    explicit SPIAcquireLock(SPISlave& slave)
        : sel((slave.configureBus(), slave))
    {
    }

private:
    SPISelectLock sel;
};

}  // namespace Boardcore
