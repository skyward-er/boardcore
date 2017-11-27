/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Silvano Seva
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

#include "spi_impl.h"

using namespace miosix;

void Spi_init()
{
    eth::sck::mode(Mode::ALTERNATE);
    eth::sck::alternateFunction(5);

    eth::miso::mode(Mode::ALTERNATE);
    eth::miso::alternateFunction(5);

    eth::mosi::mode(Mode::ALTERNATE);
    eth::mosi::alternateFunction(5);

    eth::cs::mode(Mode::OUTPUT);

    RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
    RCC_SYNC();

    // APB bus frequency is 90MHz, so leaving the BR[2:0] bits
    // in CR1 makes the SPI clock frequency to be 45MHz

    SPI5->CR1 = SPI_CR1_SSM      // CS handled in software
                | SPI_CR1_SSI    // Internal CS high
                | SPI_CR1_SPE    // SPI enabled
                | SPI_CR1_MSTR;  // Master mode
}

unsigned char Spi_sendRecv(unsigned char data)
{
    SPI5->DR = data;
    while ((SPI5->SR & SPI_SR_RXNE) == 0)
        ;  // Wait
    return SPI5->DR;
}

void Spi_CS_high() { eth::cs::high(); }

void Spi_CS_low() { eth::cs::low(); }
