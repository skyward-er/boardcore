/* Copyright (c) 2022 Skyward Experimental Rocketry
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
#include <drivers/usart/USART.h>

namespace Boardcore
{

/**
 * @brief Abstraction over the kinds of interface of the CC3135
 */
class ICC3135Iface
{
public:
    virtual void read(uint8_t *buffer, size_t size)  = 0;
    virtual void write(uint8_t *buffer, size_t size) = 0;
    virtual bool is_spi()                            = 0;
    virtual void reset()                             = 0;
};

/**
 * @brief SPI CC3135 implementation.
 */
class CC3135Spi : public ICC3135Iface
{
public:
    explicit CC3135Spi(SPIBusInterface &bus, GpioType cs,
                       SPIBusConfig config = {})
        : slave(bus, cs, config)
    {
    }

    void read(uint8_t *buffer, size_t size) override
    {
        SPITransaction spi(slave);
        spi.read(buffer, size);
    }

    void write(uint8_t *buffer, size_t size) override
    {
        SPITransaction spi(slave);
        spi.write(buffer, size);
    }

    bool is_spi() override { return true; }

    void reset() override {}

private:
    SPISlave slave;
};

/**
 * @brief UART CC3135 implementation.
 */
class CC3135Uart : public ICC3135Iface
{
public:
    explicit CC3135Uart(USARTType *usart) : usart(usart, DEFAULT_BAUD)
    {
        this->usart.init();
    }

    void read(uint8_t *buffer, size_t size) override
    {
        usart.read(buffer, size);
    }

    void write(uint8_t *buffer, size_t size) override
    {
        usart.write(buffer, size);
    }

    bool is_spi() override { return false; }

    void reset() override { usart.clearQueue(); }

private:
    static constexpr USART::Baudrate DEFAULT_BAUD = USART::Baudrate::B115200;

    USART usart;
};

}  // namespace Boardcore
