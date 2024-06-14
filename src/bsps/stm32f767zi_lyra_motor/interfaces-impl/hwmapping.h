/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#define MIOSIX_SENSOR_H3LIS331DL_SPI SPI1
#define MIOSIX_SENSOR_LPS22DF_SPI SPI1
#define MIOSIX_SENSOR_LSM6DSRX_SPI SPI3
#define MIOSIX_SENSOR_LIS2MDL_SPI SPI3
#define MIOSIX_SENSOR_ADS131M08_SPI SPI4

#define MIOSIX_SERVOS_1_TIM TIM3
#define MIOSIX_SERVOS_1_CHANNEL CHANNEL_1
#define MIOSIX_SERVOS_2_TIM TIM1
#define MIOSIX_SERVOS_2_CHANNEL CHANNEL_1

namespace miosix
{

namespace interfaces
{

// H3LIS, LPS22
namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

// LSM6, LIS2MDL
namespace spi3
{
using sck  = Gpio<GPIOB_BASE, 3>;
using miso = Gpio<GPIOB_BASE, 4>;
using mosi = Gpio<GPIOD_BASE, 6>;
}  // namespace spi3

// ADS131
namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

// Miosix UART
namespace usart1
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace usart1

// HIL UART
namespace uart4
{
using tx = Gpio<GPIOA_BASE, 0>;
using rx = Gpio<GPIOA_BASE, 1>;
}  // namespace uart4

namespace can1
{
using tx = Gpio<GPIOA_BASE, 12>;
using rx = Gpio<GPIOA_BASE, 11>;
}  // namespace can1

namespace timers
{
using tim1ch1 = Gpio<GPIOA_BASE, 8>;
using tim3ch1 = Gpio<GPIOC_BASE, 6>;
}  // namespace timers

namespace adcs
{
using adc12in9  = Gpio<GPIOB_BASE, 1>;
using adc12in14 = Gpio<GPIOC_BASE, 4>;
}  // namespace adcs

}  // namespace interfaces

namespace sensors
{

namespace ADS131M08
{
using cs = Gpio<GPIOG_BASE, 13>;
}

namespace LPS22DF
{
using cs = Gpio<GPIOG_BASE, 14>;
}

namespace LIS2MDL
{
using cs = Gpio<GPIOD_BASE, 5>;
}

namespace H3LIS331DL
{
using cs = Gpio<GPIOB_BASE, 7>;
}

namespace LSM6DSRX
{
using cs = Gpio<GPIOG_BASE, 12>;
}

}  // namespace sensors

namespace adcs
{
using vbat     = miosix::interfaces::adcs::adc12in14;
using servoCur = miosix::interfaces::adcs::adc12in9;
}  // namespace adcs

namespace gpios
{
using boardLed = Gpio<GPIOC_BASE, 5>;
}  // namespace gpios

}  // namespace miosix