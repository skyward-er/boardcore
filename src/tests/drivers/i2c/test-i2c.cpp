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

#include <iostream>

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
typedef miosix::Gpio<GPIOB_BASE, 6> i1scl1;
typedef miosix::Gpio<GPIOB_BASE, 7> i1sda1;
typedef miosix::Gpio<GPIOB_BASE, 8> i1scl2;
typedef miosix::Gpio<GPIOB_BASE, 9> i1sda2;

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
    uint8_t whoamiContent  = 0x43;
} OLED;

bool i2cDriverOLED(I2C &i2c)
{
    buffer[0] = 0;
    if (!i2c.write(OLED.addressSensor, &OLED.whoamiRegister, 1, true))
    {
        printf("writing error!\n");
        return false;
    }

    if (!i2c.read(OLED.addressSensor, buffer, 1, true))
    {
        printf("reading error!\n");
        return false;
    }
    // printf("read: %d, should be: %d\n", buffer[0], OLED.whoamiContent);
    return true;
}

bool i2cDriverBMP(I2C &i2c)
{
    i2c.write(BMP180.addressSensor, BMP180.softReset, 2, true);

    buffer[0] = 0;
    if (!i2c.write(BMP180.addressSensor, &BMP180.whoamiRegister, 1, true))
    {
        printf("writing error!\n");
        return false;
    }
    if (!i2c.read(BMP180.addressSensor, buffer, 1, true))
    {
        printf("reading error!\n");
        return false;
    }

    // printf("read: %d, should be: %d\n", buffer[0], BMP180.whoamiContent);
    return true;
}

int main()
{
    // pin settings
    i1sda2::getPin().mode(miosix::Mode::ALTERNATE);
    i1sda2::getPin().alternateFunction(4);
    i1scl2::getPin().mode(miosix::Mode::ALTERNATE);
    i1scl2::getPin().alternateFunction(4);

    I2C i2c(I2C1, I2C::Speed::STANDARD, I2C::Addressing::BIT7);

    for (;;)
    {
        bool ok = true;
        for (int i = 0; i < 30; i++)
        {
            ok &= i2cDriverOLED(i2c);
            // Thread::sleep(2);
            ok &= i2cDriverBMP(i2c);
            // Thread::sleep(10);
        }
        std::cout << std::boolalpha << ok << std::endl;
    }

    return 0;
}
