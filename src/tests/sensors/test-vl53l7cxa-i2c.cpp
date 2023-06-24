/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Adriano Longo
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

#include <iostream>

#include "drivers/i2c/I2C.h"
#include "miosix.h"
#include "string"
#include "string.h"
#include "thread"

using namespace std;
using namespace miosix;
using namespace Boardcore;

// I2C
typedef Gpio<GPIOB_BASE, 8> i1scl;
typedef Gpio<GPIOB_BASE, 9> i1sda;
typedef Gpio<GPIOB_BASE, 0> i1rst;
typedef Gpio<GPIOD_BASE, 3> pwren;
typedef Gpio<GPIOD_BASE, 5> lpn;

uint8_t address     = 0x52 / 2;
uint16_t readResult = 0x00;
I2CDriver::I2CSlaveConfig cfg{address, I2CDriver::Addressing::BIT7,
                              I2CDriver::Speed::MAX_SPEED};

void sampleContinuousMode(I2C &i2c)
{
    i1rst::getPin().high();

    delayMs(10);

    i1rst::getPin().low();

    delayMs(10);

    if (i2c.probe(cfg))
        printf("Non sono morto (ancora)\n");
    else
        printf("F\n");
}

int main()
{
    I2C i2c(I2C1, i1scl::getPin(), i1sda::getPin());

    pwren::getPin().mode(Mode::OUTPUT);
    pwren::getPin().high();

    i1rst::getPin().mode(Mode::OUTPUT);
    i1rst::getPin().low();

    lpn::getPin().mode(Mode::OUTPUT);
    lpn::getPin().high();

    for (;;)
    {
        sampleContinuousMode(i2c);
    }

    return 0;
}
