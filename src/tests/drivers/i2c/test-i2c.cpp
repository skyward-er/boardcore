/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include "drivers/i2c/I2C.h"
#include "miosix.h"
#include "string"
#include "string.h"
#include "thread"

// #define I2C_MIOSIX

#ifdef I2C_MIOSIX
#include "drivers/i2c/stm32f2_f4_i2c.h"
#endif

using namespace miosix;
using namespace Boardcore;

/**
 * SETUP:
 */

uint8_t addressSensor  = 0b1110111 << 1;
uint8_t whoamiRegister = 0xD0;
uint8_t whoamiContent  = 0x55;

uint8_t softReset[] = {
    0xE0,  // address of the software reset register
    0xB6   // write this to perform a software reset};
};
uint8_t buffer[8] = {0};
uint16_t address  = 42;

void i2cDriver()
{
#ifndef I2C_MIOSIX
    I2C i2c(I2C1, I2C::Speed::FAST, I2C::Addressing::BIT7, address);

    if (!i2c.init())
        printf("errore inizializzando i2c\n");
    else
        printf("initialized!\n");

    i2c.write(addressSensor, softReset, 2, true);

    for (;;)
    {
        buffer[0] = 0;
        if (!i2c.write(addressSensor, &whoamiRegister, 1, true))
            printf("writing error!\n");
        if (!i2c.read(addressSensor, buffer, 1, true))
            printf("reading error!\n");

        printf("read: %d, should be: %d\n", buffer[0], whoamiContent);
        miosix::Thread::sleep(1000);
    }
#else
    miosix::I2C1Driver &i2c = miosix::I2C1Driver::instance();
    while (true)
    {
        if (!i2c.send(addressSensor, &whoamiRegister, 1))
            printf("writing error!\n");

        if (!i2c.recv(addressSensor, buffer, 1))
            printf("reading error!\n");

        printf("read: %d, should be: %d\n", buffer[0], whoamiContent);

        miosix::Thread::sleep(1000);
    }
#endif
    miosix::Thread::sleep(1000);
}

int main()
{
    // pin settings
    i1sda2::getPin().mode(miosix::Mode::ALTERNATE);
    i1sda2::getPin().alternateFunction(4);
    i1scl2::getPin().mode(miosix::Mode::ALTERNATE);
    i1scl2::getPin().alternateFunction(4);

    i2cDriver();

    return 0;
}
