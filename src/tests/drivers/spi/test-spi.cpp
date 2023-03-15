/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#include <drivers/spi/SPIDriver.h>

#include <cstdio>
#include <iostream>
#include <sstream>

using namespace Boardcore;
using namespace std;
using namespace miosix;

SPIBus bus(SPI1);

GpioPin sckPin(GPIOA_BASE, 5);
GpioPin misoPin(GPIOA_BASE, 6);
GpioPin mosiPin(GPIOA_BASE, 7);
GpioPin csPin(GPIOA_BASE, 3);

int main()
{
    sckPin.mode(Mode::ALTERNATE);
    sckPin.alternateFunction(5);
    sckPin.speed(Speed::_100MHz);
    misoPin.mode(Mode::ALTERNATE);
    misoPin.alternateFunction(5);
    mosiPin.mode(Mode::ALTERNATE);
    mosiPin.alternateFunction(5);
    csPin.mode(Mode::OUTPUT);
    csPin.high();

    SPISlave spiSlave(bus, csPin);
    spiSlave.config.clockDivider = SPI::ClockDivider::DIV_64;

    SPITransaction transaction(spiSlave);

    uint8_t buffer8[6];
    uint16_t buffer16[6];

    // Reads
    {
        transaction.read();
        delayMs(1);
        transaction.read16();
        delayMs(1);
        transaction.read24();
        delayMs(1);
        transaction.read32();
        delayMs(1);
        transaction.read(buffer8, sizeof(buffer8));
        delayMs(1);
        transaction.read16(buffer16, sizeof(buffer16));
        delayMs(1);
    }

    // Writes
    {
        transaction.write((uint8_t)0xAB);
        delayMs(1);
        transaction.write16((uint16_t)0xABCD);
        delayMs(1);
        transaction.write24((uint32_t)0xABCDEF);
        delayMs(1);
        transaction.write32((uint32_t)0xABCDEF01);
        delayMs(1);
        buffer8[0] = 0x01;
        buffer8[1] = 0x23;
        buffer8[2] = 0x45;
        buffer8[3] = 0x67;
        buffer8[4] = 0x89;
        buffer8[5] = 0xAB;
        transaction.write(buffer8, sizeof(buffer8));
        delayMs(1);
        buffer16[0] = 0x0101;
        buffer16[1] = 0x2323;
        buffer16[2] = 0x4545;
        buffer16[3] = 0x6767;
        buffer16[4] = 0x8989;
        buffer16[5] = 0xABAB;
        transaction.write16(buffer16, sizeof(buffer16));
        delayMs(1);
    }

    // Transaction
    {
        transaction.transfer((uint8_t)0xAB);
        delayMs(1);
        transaction.transfer16((uint16_t)0xABCD);
        delayMs(1);
        transaction.transfer24((uint32_t)0xABCDEF);
        delayMs(1);
        transaction.transfer32((uint32_t)0xABCDEF01);
        delayMs(1);
        transaction.transfer(buffer8, sizeof(buffer8));
        delayMs(1);
        transaction.transfer16(buffer16, sizeof(buffer16));
        delayMs(1);
    }

    // Registers
    {
        transaction.readRegister(0x12);
        delayMs(1);
        transaction.readRegister16(0x34);
        delayMs(1);
        transaction.readRegister24(0x56);
        delayMs(1);
        transaction.writeRegister(0x12, 0xAB);
        delayMs(1);
        transaction.writeRegister16(0x34, 0xABCD);
        delayMs(1);
        transaction.writeRegister24(0x56, 0xABCDEF);
        delayMs(1);
    }

    while (true)
        delayMs(1000);
}
