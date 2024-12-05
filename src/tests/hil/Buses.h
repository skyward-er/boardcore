/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <drivers/i2c/I2C.h>
#include <drivers/spi/SPIBus.h>
#include <drivers/usart/USART.h>
#include <interfaces-impl/hwmapping.h>

#include <utils/ModuleManager/ModuleManager.hpp>
namespace HILTest
{
class Buses : public Boardcore::Module
{
public:
    Boardcore::SPIBus spi1;
    Boardcore::SPIBus spi3;
    Boardcore::SPIBus spi4;
    Boardcore::SPIBus spi6;

    Boardcore::I2C i2c1;

    Boardcore::USART usart1;
    Boardcore::USART usart2;
    Boardcore::USART uart4;

    Buses()
        : spi1(SPI1), spi3(SPI3), spi4(SPI4), spi6(SPI6),
          i2c1(I2C1, miosix::interfaces::i2c1::scl::getPin(),
               miosix::interfaces::i2c1::sda::getPin()),
          usart1(USART1, 115200), usart2(USART2, 256000, 1024),
          uart4(UART4, 115200)
    {
    }
};
}  // namespace HILTest
