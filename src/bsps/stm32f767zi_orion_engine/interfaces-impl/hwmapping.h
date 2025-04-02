/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Davide Mor, Fabrizio Monti
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
using tim1ch1  = Gpio<GPIOA_BASE, 8>;
using tim3ch1  = Gpio<GPIOC_BASE, 6>;
using tim4ch1  = Gpio<GPIOD_BASE, 12>;
using tim8ch2  = Gpio<GPIOC_BASE, 7>;
using tim9ch1  = Gpio<GPIOA_BASE, 2>;
using tim11ch1 = Gpio<GPIOB_BASE, 9>;
using tim12ch2 = Gpio<GPIOB_BASE, 15>;
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

namespace LSM6DSRX0
{
using cs = Gpio<GPIOG_BASE, 12>;
}

namespace LSM6DSRX1
{
using cs = Gpio<GPIOG_BASE, 11>;
}

namespace thermocouple
{
using cs = Gpio<GPIOD_BASE, 11>;
}

}  // namespace sensors

namespace adcs
{
using vbat     = miosix::interfaces::adcs::adc12in14;
using servoCur = miosix::interfaces::adcs::adc12in9;
}  // namespace adcs

namespace servos
{
#define MIOSIX_SERVOS_0_TIM TIM3
#define MIOSIX_SERVOS_0_CHANNEL CHANNEL_1
using servo0 = miosix::interfaces::timers::tim3ch1;

#define MIOSIX_SERVOS_1_TIM TIM1
#define MIOSIX_SERVOS_1_CHANNEL CHANNEL_1
using servo1 = miosix::interfaces::timers::tim1ch1;

#define MIOSIX_SERVOS_2_TIM TIM8
#define MIOSIX_SERVOS_2_CHANNEL CHANNEL_2
using servo2 = miosix::interfaces::timers::tim8ch2;

#define MIOSIX_SERVOS_3_TIM TIM9
#define MIOSIX_SERVOS_3_CHANNEL CHANNEL_1
using servo3 = miosix::interfaces::timers::tim9ch1;

#define MIOSIX_SERVOS_4_TIM TIM11
#define MIOSIX_SERVOS_4_CHANNEL CHANNEL_1
using servo4 = miosix::interfaces::timers::tim11ch1;

#define MIOSIX_SERVOS_5_TIM TIM4
#define MIOSIX_SERVOS_5_CHANNEL CHANNEL_1
using servo5 = miosix::interfaces::timers::tim4ch1;
}  // namespace servos

namespace gpios
{
using boardLed = Gpio<GPIOC_BASE, 5>;
}  // namespace gpios

}  // namespace miosix
