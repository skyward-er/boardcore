/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Riccardo Sironi
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

#include "PCA9685.h"

#include <miosix.h>
#include <utils/Debug.h>

#include <bitset>
#include <iostream>

#include "drivers/i2c/I2CDriver.h"

// Based on the board schematics, the following pins are used for SPI
typedef miosix::Gpio<GPIOB_BASE, 8> sck;
typedef miosix::Gpio<GPIOB_BASE, 9> sda;

float setPosition(float position, bool limited);

int main()
{
    /* Initialize the I2C bus and the pins for SCK, SDA */
    sck::mode(miosix::Mode::ALTERNATE_OD);
    sck::alternateFunction(4);  // I2C1_SCL
    sda::mode(miosix::Mode::ALTERNATE_OD);
    sda::alternateFunction(4);  // I2C1_SDA

    // Create an instance of I2C Bus Interface
    Boardcore::I2C bus(I2C1, sck::getPin(), sda::getPin());

    Boardcore::I2CDriver::I2CSlaveConfig PCA9685Config{
        0b1001100, Boardcore::I2CDriver::Addressing::BIT7,
        Boardcore::I2CDriver::Speed::MAX_SPEED};

    // Create an instance of the PCA9685 sensor
    Boardcore::PCA9685 pca9685(bus, PCA9685Config, static_cast<uint8_t>(0x11));

    // Initialize the sensor
    if (!pca9685.init())
    {
        std::cout << "Error initializing the PCA9685 Controller" << std::endl;
        std::cout << "Last error code: "
                  << static_cast<int>(pca9685.getLastError()) << std::endl;

        return 1;
    }
    // printf("PCA9685 initialized successfully\n");
    while (true)
    {
        if (!pca9685.setAllDutyCycle(20))
        {
            std::cout << "Error setting the Duty Cycle a 0" << std::endl;
            std::cout << "Last error code: "
                      << static_cast<int>(pca9685.getLastError()) << std::endl;

            return 1;
        }

        // if (!pca9685.setPWM(Boardcore::PCA9685::Channels::CHANNEL_5, 0, 500))
        // {
        //     std::cout << "Error setting the Duty Cycle" << std::endl;
        //     std::cout << "Last error code: "
        //               << static_cast<int>(pca9685.getLastError()) <<
        //               std::endl;

        //     return 1;
        // }
        // if (!pca9685.setDutyCycle(Boardcore::PCA9685::Channels::CHANNEL_0,
        //                           setPosition(0.5f, true)))
        // {
        //     std::cout << "Error setting the Duty Cycle" << std::endl;
        //     std::cout << "Last error code: "
        //               << static_cast<int>(pca9685.getLastError()) <<
        //               std::endl;

        //     return 1;
        // }

        // miosix::Thread::sleep(3000);

        // // animate the sensor
        // if (!pca9685.setPWM(Boardcore::PCA9685::Channels::CHANNEL_5, 0,
        // 2700))
        // {
        //     std::cout << "Error setting the Duty Cycle" << std::endl;
        //     std::cout << "Last error code: "
        //               << static_cast<int>(pca9685.getLastError()) <<
        //               std::endl;

        //     return 1;
        // }
        // if (!pca9685.setDutyCycle(Boardcore::PCA9685::Channels::CHANNEL_0,
        //                           setPosition(0.3f, true)))
        // {
        //     std::cout << "Error setting the Duty Cycle" << std::endl;
        //     std::cout << "Last error code: "
        //               << static_cast<int>(pca9685.getLastError()) <<
        //               std::endl;

        //     return 1;
        // }
        miosix::Thread::sleep(3000);
    }
    return 0;
}

// custom function, copied from Servo.h
float setPosition(float position, bool limited)
{
    if (limited)
    {
        if (position < 0)
            position = 0;
        else if (position > 1)
            position = 1;
    }

    float pulse = 500 + (2440 - 500) * position;

    float dutyCycle = pulse * 333.0f / 1000000.0f;

    return dutyCycle * 100;
}
