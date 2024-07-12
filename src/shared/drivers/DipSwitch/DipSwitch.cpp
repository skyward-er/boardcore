/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Nicolò Caruso
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

#include "DipSwitch.h"

uint8_t DipSwitch::read()
{
    uint8_t read = 0;

    // Write to the shift register (CS == Not LD)
    sh.low();
    miosix::delayUs(microSecClk);
    clk.high();
    miosix::delayUs(microSecClk);
    sh.high();
    miosix::delayUs(microSecClk);

    // Read first register GS(0)/ARP(1)

    read |= readBit();
    read |= readBit() << 1;
    read |= readBit() << 2;
    read |= readBit() << 3;
    read |= readBit() << 4;
    read |= readBit() << 5;
    read |= readBit() << 6;
    read |= readBit() << 7;

    return read;
}

uint8_t DipSwitch::readBit()
{
    uint8_t bit;
    clk.high();
    miosix::delayUs(microSecClk);
    bit = qh.value();
    miosix::delayUs(microSecClk);
    clk.low();
    miosix::delayUs(microSecClk);
    return bit;
}