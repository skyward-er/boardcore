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

// I2C1
typedef miosix::Gpio<GPIOB_BASE, 6> i1sda1;
typedef miosix::Gpio<GPIOB_BASE, 7> i1scl1;
typedef miosix::Gpio<GPIOB_BASE, 8> i1sda2;
typedef miosix::Gpio<GPIOB_BASE, 9> i1scl2;

// I2C2
typedef miosix::Gpio<GPIOB_BASE, 9> i2sda1;
typedef miosix::Gpio<GPIOB_BASE, 10> i2scl1;
typedef miosix::Gpio<GPIOB_BASE, 11> i2sda2;
typedef miosix::Gpio<GPIOB_BASE, 12> i2scl2;
#ifndef STM32F401xE
typedef miosix::Gpio<GPIOF_BASE, 0> i2sda3;
typedef miosix::Gpio<GPIOF_BASE, 1> i2scl3;
typedef miosix::Gpio<GPIOH_BASE, 4> i2sda4;
typedef miosix::Gpio<GPIOH_BASE, 5> i2scl4;
#endif

// I2C3
typedef miosix::Gpio<GPIOC_BASE, 9> i3sda1;
typedef miosix::Gpio<GPIOA_BASE, 8> i3scl1;
#ifndef STM32F401xE
typedef miosix::Gpio<GPIOH_BASE, 7> i3sda2;
typedef miosix::Gpio<GPIOH_BASE, 8> i3scl2;
#endif

/**
 * SETUP:
 */

uint16_t address = 42;

uint8_t buffer[8] = {0};
struct
{
    uint8_t addressSensor  = 0b1110111;
    uint8_t whoamiRegister = 0xD0;
    uint8_t whoamiContent  = 0x55;
    uint8_t softReset[2]   = {
          0xE0,  // address of the software reset register
          0xB6   // write this to perform a software reset};
    };
} BMP180;

struct
{
    uint8_t addressSensor  = 0b0111100;
    uint8_t whoamiRegister = 0xD0;
    uint8_t whoamiContent  = 0x55;
} OLED;

void i2cDriverOLED()
{
#ifndef I2C_MIOSIX
    I2C i2c(I2C1, I2C::Speed::STANDARD, I2C::Addressing::BIT7, address);

    if (!i2c.init())
        printf("errore inizializzando i2c\n");
    else
        printf("initialized!\n");

    // pin settings
    i1sda2::getPin().mode(miosix::Mode::ALTERNATE);
    i1sda2::getPin().alternateFunction(4);
    i1scl2::getPin().mode(miosix::Mode::ALTERNATE);
    i1scl2::getPin().alternateFunction(4);

    for (;;)
    {
        buffer[0] = 0;
        if (!i2c.write(OLED.addressSensor, &OLED.whoamiRegister, 1, true))
            printf("writing error!\n");
        if (!i2c.read(OLED.addressSensor, buffer, 1, true))
            printf("reading error!\n");

        printf("read: %d\n", buffer[0]);
        miosix::Thread::sleep(1000);
    }
#else
    miosix::I2C1Driver &i2c = miosix::I2C1Driver::instance();
    while (true)
    {
        if (!i2c.send(OLED.addressSensor, &OLED.whoamiRegister, 1))
            printf("writing error!\n");

        if (!i2c.recv(OLED.addressSensor, buffer, 1))
            printf("reading error!\n");

        printf("read: %d\n", buffer[0]);

        miosix::Thread::sleep(1000);
    }
#endif
    miosix::Thread::sleep(1000);
}

void i2cDriver()
{
#ifndef I2C_MIOSIX
    I2C i2c(I2C1, I2C::Speed::STANDARD, I2C::Addressing::BIT7, address);

    if (!i2c.init())
        printf("errore inizializzando i2c\n");
    else
        printf("initialized!\n");

    i2c.write(BMP180.addressSensor, BMP180.softReset, 2, true);

    for (;;)
    {
        buffer[0] = 0;
        if (!i2c.write(BMP180.addressSensor, &BMP180.whoamiRegister, 1, true))
            printf("writing error!\n");
        if (!i2c.read(BMP180.addressSensor, buffer, 1, true))
            printf("reading error!\n");

        printf("read: %d, should be: %d\n", buffer[0], BMP180.whoamiContent);
        miosix::Thread::sleep(1000);
    }
#else
    miosix::I2C1Driver &i2c = miosix::I2C1Driver::instance();
    while (true)
    {
        if (!i2c.send(BMP180.addressSensor, &BMP180.whoamiRegister, 1))
            printf("writing error!\n");

        if (!i2c.recv(BMP180.addressSensor, buffer, 1))
            printf("reading error!\n");

        printf("read: %d, should be: %d\n", buffer[0], BMP180.whoamiContent);

        miosix::Thread::sleep(1000);
    }
#endif
    miosix::Thread::sleep(1000);
}

int main()
{
    i2cDriverOLED();

    return 0;
}
