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

#include "drivers/i2c/I2CDriver.h"
#include "miosix.h"
#include "scheduler/TaskScheduler.h"
#include "string"
#include "string.h"
#include "thread"

using namespace std;
using namespace miosix;
using namespace Boardcore;

bool generateStop = false;

// I2CDriver1
typedef miosix::Gpio<GPIOB_BASE, 6> i1scl1;
typedef miosix::Gpio<GPIOB_BASE, 7> i1sda1;
typedef miosix::Gpio<GPIOB_BASE, 8> i1scl2;
typedef miosix::Gpio<GPIOB_BASE, 9> i1sda2;

// I2CDriver2
typedef miosix::Gpio<GPIOB_BASE, 9> i2sda1;
typedef miosix::Gpio<GPIOB_BASE, 10> i2scl1;
typedef miosix::Gpio<GPIOB_BASE, 11> i2sda2;
typedef miosix::Gpio<GPIOB_BASE, 12> i2scl2;
#ifdef GPIOH
typedef miosix::Gpio<GPIOF_BASE, 0> i2sda3;
typedef miosix::Gpio<GPIOF_BASE, 1> i2scl3;
typedef miosix::Gpio<GPIOH_BASE, 4> i2sda4;
typedef miosix::Gpio<GPIOH_BASE, 5> i2scl4;
#endif

// I2CDriver3
typedef miosix::Gpio<GPIOC_BASE, 9> i3sda1;
typedef miosix::Gpio<GPIOA_BASE, 8> i3scl1;
#ifdef GPIOH
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
    uint8_t softReset[2]   = {0xE0, 0xB6};
} BMP180;

struct
{
    uint8_t addressSensor  = 0b1110110;
    uint8_t whoamiRegister = 0xD0;
    uint8_t whoamiContent  = 0x60;
    uint8_t softReset[2]   = {0xE0, 0xB6};
} BME280;

struct
{
    uint8_t addressSensor  = 0b0111100;
    uint8_t whoamiRegister = 0xD0;
    uint8_t whoamiContent  = 0x43;
} OLED;

bool i2cDriverOLED(I2CDriver &i2c)
{
    buffer[0] = 0;
    if (!i2c.write(OLED.addressSensor, &OLED.whoamiRegister, 1) ||
        !i2c.read(OLED.addressSensor, buffer, 1) ||
        buffer[0] != OLED.whoamiContent)
    {
        return false;
    }

    return true;
}

bool i2cDriverBMP(I2CDriver &i2c)
{
    buffer[0] = 0;
    if (!i2c.write(BMP180.addressSensor, BMP180.softReset, 2) ||
        !i2c.write(BMP180.addressSensor, &BMP180.whoamiRegister, 1,
                   generateStop) ||
        !i2c.read(BMP180.addressSensor, buffer, 1) ||
        buffer[0] != BMP180.whoamiContent)
    {
        return false;
    }

    return true;
}

bool i2cDriverBME(I2CDriver &i2c)
{
    buffer[0] = 0;
    if (!i2c.write(BME280.addressSensor, BME280.softReset, 2) ||
        !i2c.write(BME280.addressSensor, &BME280.whoamiRegister, 1,
                   generateStop) ||
        !i2c.read(BME280.addressSensor, buffer, 1) ||
        buffer[0] != BME280.whoamiContent)
    {
        return false;
    }

    return true;
}

int main()
{
    int nRepeat = 50;

    I2CDriver i2c(I2C1, I2CDriver::Speed::STANDARD, I2CDriver::Addressing::BIT7,
                  i1scl2::getPin(), i1sda2::getPin());

    // scheduling the flush of the I2CDriver bus
    TaskScheduler scheduler;
    scheduler.addTask([&]() { i2c.flushBus(); }, 100);
    scheduler.start();

    for (;;)
    {
        // resetting status of read sensors
        bool statusOLED = true;
        bool statusBMP  = true;

        for (int i = 0; i < nRepeat; i++)
        {
            statusOLED &= i2cDriverOLED(i2c);
            statusBMP &= i2cDriverBMP(i2c);
        }

        printf("OLED:%d\tBMP:%d\n", statusOLED, statusBMP);
    }
    return 0;
}
