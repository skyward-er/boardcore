/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#pragma once

#include "interfaces/gpio.h"

namespace miosix
{
namespace interfaces
{
namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

namespace spi3
{
using sck  = Gpio<GPIOB_BASE, 3>;
using miso = Gpio<GPIOB_BASE, 4>;
using mosi = Gpio<GPIOD_BASE, 6>;
}  // namespace spi3

namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

namespace spi6
{
using sck  = Gpio<GPIOG_BASE, 13>;
using miso = Gpio<GPIOG_BASE, 12>;
using mosi = Gpio<GPIOG_BASE, 14>;
}  // namespace spi6

namespace i2c1
{
using sda = Gpio<GPIOB_BASE, 9>;
using scl = Gpio<GPIOB_BASE, 8>;
}  // namespace i2c1

namespace can1
{
using rx = Gpio<GPIOA_BASE, 11>;
using tx = Gpio<GPIOA_BASE, 12>;
}  // namespace can1

namespace can2
{
using rx = Gpio<GPIOB_BASE, 12>;
using tx = Gpio<GPIOB_BASE, 13>;
}  // namespace can2

namespace usart1
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace usart1

namespace usart2
{
using tx = Gpio<GPIOA_BASE, 2>;
using rx = Gpio<GPIOA_BASE, 3>;
}  // namespace usart2

namespace uart4
{
using tx = Gpio<GPIOA_BASE, 0>;
using rx = Gpio<GPIOA_BASE, 1>;
}  // namespace uart4

namespace timers
{
using tim3ch1  = Gpio<GPIOC_BASE, 6>;   // Airbrakes servo   - Servo1 Payload
using tim3ch2  = Gpio<GPIOC_BASE, 7>;   // Auxiliary         - Servo2 Payload
using tim1ch1  = Gpio<GPIOA_BASE, 8>;   // Buzzer
using tim12ch2 = Gpio<GPIOB_BASE, 15>;  // Expulsion
}  // namespace timers

}  // namespace interfaces

namespace sensors
{
namespace LSM6DSRX
{
using sck        = interfaces::spi1::sck;
using miso       = interfaces::spi1::miso;
using mosi       = interfaces::spi1::mosi;
using cs         = Gpio<GPIOC_BASE, 4>;
using interrupt1 = Gpio<GPIOD_BASE, 13>;
using interrupt2 = Gpio<GPIOG_BASE, 7>;
}  // namespace LSM6DSRX

namespace H3LIS331DL
{
using sck  = interfaces::spi3::sck;
using miso = interfaces::spi3::miso;
using mosi = interfaces::spi3::mosi;
using cs   = Gpio<GPIOD_BASE, 3>;
}  // namespace H3LIS331DL

namespace LIS2MDL
{
using sck  = interfaces::spi3::sck;
using miso = interfaces::spi3::miso;
using mosi = interfaces::spi3::mosi;
using cs   = Gpio<GPIOD_BASE, 5>;
}  // namespace LIS2MDL

namespace LPS22DF
{
using sck       = interfaces::spi3::sck;
using miso      = interfaces::spi3::miso;
using mosi      = interfaces::spi3::mosi;
using cs        = Gpio<GPIOD_BASE, 7>;
using interrupt = Gpio<GPIOB_BASE, 11>;
}  // namespace LPS22DF

namespace GPS
{
using sck  = interfaces::spi4::sck;
using miso = interfaces::spi4::miso;
using mosi = interfaces::spi4::mosi;
using cs   = Gpio<GPIOE_BASE, 4>;
}  // namespace GPS

namespace VN100
{
using sck  = interfaces::spi4::sck;
using miso = interfaces::spi4::miso;
using mosi = interfaces::spi4::mosi;
using cs   = Gpio<GPIOB_BASE, 14>;
}  // namespace VN100

namespace VN100_SERIAL
{
using tx = interfaces::uart4::tx;
using rx = interfaces::uart4::rx;
}  // namespace VN100_SERIAL

namespace ADS131
{
using sck  = interfaces::spi4::sck;
using miso = interfaces::spi4::miso;
using mosi = interfaces::spi4::mosi;
using cs   = Gpio<GPIOG_BASE, 10>;
}  // namespace ADS131

namespace LPS28DFW_1
{
using sda = interfaces::i2c1::sda;
using scl = interfaces::i2c1::scl;
}  // namespace LPS28DFW_1

namespace LPS28DFW_2
{
using sda = interfaces::i2c1::sda;
using scl = interfaces::i2c1::scl;
}  // namespace LPS28DFW_2

}  // namespace sensors

namespace radio
{
using sck       = interfaces::spi6::sck;
using miso      = interfaces::spi6::miso;
using mosi      = interfaces::spi6::mosi;
using cs        = Gpio<GPIOG_BASE, 11>;
using dio0      = Gpio<GPIOC_BASE, 3>;
using dio1      = Gpio<GPIOD_BASE, 4>;
using dio3      = Gpio<GPIOC_BASE, 5>;
using rx_enable = Gpio<GPIOB_BASE, 0>;
using tx_enable = Gpio<GPIOC_BASE, 1>;
}  // namespace radio

namespace actuators
{
using airbrakes       = interfaces::timers::tim3ch1;
using expulsion       = interfaces::timers::tim12ch2;
using buzzer          = interfaces::timers::tim1ch1;
using parafoil_servo1 = interfaces::timers::tim3ch1;
using parafoil_servo2 = interfaces::timers::tim3ch2;
}  // namespace actuators

namespace gpios
{
using cut_trigger     = Gpio<GPIOA_BASE, 15>;
using cut_sense       = Gpio<GPIOD_BASE, 12>;
using exp_enable      = Gpio<GPIOB_BASE, 1>;
using status_led      = Gpio<GPIOA_BASE, 14>;
using camera_enable   = Gpio<GPIOA_BASE, 12>;
using liftoff_detach  = Gpio<GPIOA_BASE, 11>;
using nosecone_detach = Gpio<GPIOA_BASE, 4>;
using exp_sense       = Gpio<GPIOG_BASE, 6>;

}  // namespace gpios

}  // namespace miosix