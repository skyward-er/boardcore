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
#include <diagnostic/CpuMeter/CpuMeter.h>

#include <iostream>

#include "drivers/i2c/I2CDriver.h"
#include "miosix.h"
#include "string"
#include "string.h"
#include "thread"

using namespace std;
using namespace miosix;
using namespace Boardcore;

bool generateStop = false;

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
I2CSensor LPS{0b1011100, 0xF, 0xB4, {}};

I2CDriver::I2CSlaveConfig BMP180Config{BMP180.addressSensor,
                                       I2CDriver::Addressing::BIT7,
                                       I2CDriver::Speed::STANDARD};

I2CDriver::I2CSlaveConfig BME280Config{BME280.addressSensor,
                                       I2CDriver::Addressing::BIT7,
                                       I2CDriver::Speed::STANDARD};

I2CDriver::I2CSlaveConfig OLEDConfig{OLED.addressSensor,
                                     I2CDriver::Addressing::BIT7,
                                     I2CDriver::Speed::STANDARD};

I2CDriver::I2CSlaveConfig LPSConfig{
    LPS.addressSensor, I2CDriver::Addressing::BIT7, I2CDriver::Speed::STANDARD};

bool i2cDriver(I2CDriver &i2c, I2CSensor sensor,
               I2CDriver::I2CSlaveConfig sensorConfig)
{
    const size_t nRead    = 300;
    uint8_t buffer[nRead] = {0};

    i2c.flushBus();

    // reset the sensor and then read the whoami
    if (!(i2c.write(sensorConfig, sensor.softReset, 2) &&
          i2c.write(sensorConfig, &sensor.whoamiRegister, 1, false) &&
          i2c.read(sensorConfig, buffer, nRead)))
    {
        uint16_t lastError{i2c.getLastError()};
        if (!(lastError &
              (I2CDriver::Errors::AF | I2CDriver::Errors::BUS_LOCKED)))
        {
            printf("LastError: %d\n", lastError);
        }
        return false;
    }

    if (buffer[0] != sensor.whoamiContent)
    {
        printf("whoami expected %d, received %d\n", sensor.whoamiContent,
               buffer[0]);
        return false;
    }

    return true;
}

int main()
{
    I2CDriver i2c(I2C1, i1scl2::getPin(), i1sda2::getPin());

    int nRepeat = 50;

    // // thread that uses 100% CPU
    // std::thread t(
    //     []()
    //     {
    //         while (1)
    //             ;
    //     });

    for (;;)
    {
        // resetting status of read sensors
        bool statusOLED = true;
        bool statusBMP  = true;
        bool statusLPS  = true;

        for (int i = 0; i < nRepeat; i++)
        {
            for (I2CDriver::Speed speed :
                 {I2CDriver::Speed::STANDARD, I2CDriver::Speed::FAST
#ifdef _ARCH_CORTEXM7_STM32F7
                  ,
                  I2CDriver::Speed::FAST_PLUS
#endif  // _ARCH_CORTEXM7_STM32F7
                 })
            {
                statusOLED &= i2cDriver(
                    i2c, OLED,
                    {OLED.addressSensor, I2CDriver::Addressing::BIT7, speed});
                statusBMP &= i2cDriver(
                    i2c, BMP180,
                    {BMP180.addressSensor, I2CDriver::Addressing::BIT7, speed});
                statusLPS &= i2cDriver(
                    i2c, LPS,
                    {LPS.addressSensor, I2CDriver::Addressing::BIT7, speed});
            }
        }

        printf("CPU: %5.1f OLED:%d BMP:%d LPS:%d\n",
               CpuMeter::getCpuStats().mean, statusOLED, statusBMP, statusLPS);
    }
    return 0;
}
