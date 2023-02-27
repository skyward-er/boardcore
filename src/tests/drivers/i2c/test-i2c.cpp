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
#include "scheduler/TaskScheduler.h"
#include "string"
#include "string.h"
#include "thread"

using namespace std;
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
#ifdef GPIOH
typedef miosix::Gpio<GPIOF_BASE, 0> i2sda3;
typedef miosix::Gpio<GPIOF_BASE, 1> i2scl3;
typedef miosix::Gpio<GPIOH_BASE, 4> i2sda4;
typedef miosix::Gpio<GPIOH_BASE, 5> i2scl4;
#endif

// I2C3
typedef miosix::Gpio<GPIOC_BASE, 9> i3sda1;
typedef miosix::Gpio<GPIOA_BASE, 8> i3scl1;
#ifdef GPIOH
typedef miosix::Gpio<GPIOH_BASE, 7> i3sda2;
typedef miosix::Gpio<GPIOH_BASE, 8> i3scl2;
#endif

/**
 * SETUP: Connect to the I2C1 port a pullup circuit and than sensors of your
 * choice (in this test there are the data for some random sensors). The test
 * just tries to write and read from these sensors. In order to test the
 * flushBus method try to disconnect and reconnect rapidly the SCL connection of
 * one sensor (in order to provoke a locked state).
 */

uint8_t buffer = 0;

typedef struct
{
    // cppcheck-suppress unusedStructMember
    const uint8_t addressSensor;
    const uint8_t whoamiRegister;
    const uint8_t whoamiContent;
    const uint8_t softReset[2];
} I2CSensor;

I2CSensor BMP180{0b1110111, 0xD0, 0x55, {0xE0, 0xB6}};
I2CSensor BME280{0b1110110, 0xD0, 0x60, {0xE0, 0xB6}};
I2CSensor OLED{0b0111100, 0xD0, 0x43, {}};

I2CDriver::I2CSlaveConfig BMP180Config{BMP180.addressSensor,
                                       I2CDriver::Addressing::BIT7,
                                       I2CDriver::Speed::STANDARD};

I2CDriver::I2CSlaveConfig BME280Config{BME280.addressSensor,
                                       I2CDriver::Addressing::BIT7,
                                       I2CDriver::Speed::STANDARD};

I2CDriver::I2CSlaveConfig OLEDConfig_F{
    OLED.addressSensor, I2CDriver::Addressing::BIT7, I2CDriver::Speed::FAST};

#ifdef _ARCH_CORTEXM7_STM32F7
I2CDriver::I2CSlaveConfig OLEDConfig_FP{OLED.addressSensor,
                                        I2CDriver::Addressing::BIT7,
                                        I2CDriver::Speed::FAST_PLUS};
#endif  // _ARCH_CORTEXM7_STM32F7

bool i2cDriver(I2C &i2c, I2CSensor sensor,
               I2CDriver::I2CSlaveConfig sensorConfig)
{
    buffer = 0;

    // reset the sensor and then read the whoami
    if (!(i2c.probe(sensorConfig) &&
          i2c.write(sensorConfig, sensor.softReset, 2) &&
          i2c.readRegister(sensorConfig, sensor.whoamiRegister, buffer)))
    {
        uint16_t lastError{i2c.getLastError()};
        if (!(lastError &
              (I2CDriver::Errors::AF | I2CDriver::Errors::BUS_LOCKED)))
        {
            printf("LastError: %d\n", lastError);
        }
        return false;
    }

    if (buffer != sensor.whoamiContent)
    {
        printf("whoami expected %d, received %d\n", sensor.whoamiContent,
               buffer);
        return false;
    }

    return true;
}

int main()
{
    int nRepeat = 50;

    // thread that uses 100% CPU
    std::thread t(
        []()
        {
            while (1)
                ;
        });

    for (;;)
    {
        SyncedI2C i2c(I2C1, i1scl2::getPin(), i1sda2::getPin());

        // resetting status of read sensors
        bool statusOLED = true;
        bool statusBMP  = true;

        for (int i = 0; i < nRepeat; i++)
        {
            statusBMP &= i2cDriver(i2c, BMP180, BMP180Config);
            statusOLED &= i2cDriver(i2c, OLED, OLEDConfig_F);
#ifdef _ARCH_CORTEXM7_STM32F7
            statusOLED &= i2cDriver(i2c, OLED, OLEDConfig_FP);
#endif  // _ARCH_CORTEXM7_STM32F7
        }

        printf("OLED:%d\tBMP:%d\n", statusOLED, statusBMP);
    }
    return 0;
}
