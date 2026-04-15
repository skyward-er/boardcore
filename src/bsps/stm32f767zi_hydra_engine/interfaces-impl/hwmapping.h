/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Davide Mor, Fabrizio Monti, Riccardo Sironi
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

#define MIOSIX_SENSOR_ADS131M08_1_SPI SPI3
#define MIOSIX_SENSOR_ADS131M08_2_SPI SPI4

namespace miosix
{

namespace interfaces
{
// ADS131
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

namespace i2c1
{
using scl = Gpio<GPIOB_BASE, 8>;
using sda = Gpio<GPIOB_BASE, 7>;
}  // namespace i2c1

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
using tim13ch1 = Gpio<GPIOA_BASE, 6>;
using tim14ch1 = Gpio<GPIOA_BASE, 7>;
}  // namespace timers

namespace adcs
{
using adc12in9  = Gpio<GPIOB_BASE, 1>;
using adc12in14 = Gpio<GPIOC_BASE, 4>;
}  // namespace adcs

}  // namespace interfaces

namespace sensors
{
namespace ADC_1
{
using cs = Gpio<GPIOG_BASE, 9>;
}

namespace ADC_2
{
using cs = Gpio<GPIOE_BASE, 4>;
}

}  // namespace sensors

namespace adcs
{
using vbat     = miosix::interfaces::adcs::adc12in14;
using servoCur = miosix::interfaces::adcs::adc12in9;
}  // namespace adcs

namespace servos
{

#define MIOSIX_OX_VENTING_VALVE_TIM TIM11
#define MIOSIX_OX_VENTING_VALVE_CHANNEL CHANNEL_1
using ventOx = miosix::interfaces::timers::tim11ch1;

#define MIOSIX_FUEL_VENTING_VALVE_CHANNEL CHANNEL_1
#define MIOSIX_FUEL_VENTING_VALVE_TIM TIM9
using ventingFuel = miosix::interfaces::timers::tim9ch1;

// Extra Valves that are currently not in use.
// #define MIOSIX_EXTRA_TOP_VALVE_TIM TIM13
// #define MIOSIX_EXTRA_TOP_VALVE_CHANNEL CHANNEL_1
// using extraTop = miosix::interfaces::timers::tim13ch1;

// #define MIOSIX_EXTRA_BOTTOM_VALVE_TIM TIM14
// #define MIOSIX_EXTRA_BOTTOM_VALVE_CHANNEL CHANNEL_1
// using extraBot = miosix::interfaces::timers::tim14ch1;

#define MIOSIX_MAIN_OX_VALVE_TIM TIM3
#define MIOSIX_MAIN_OX_VALVE_CHANNEL CHANNEL_1
using mainOx = miosix::interfaces::timers::tim3ch1;

#define MIOSIX_MAIN_FUEL_VALVE_TIM TIM8
#define MIOSIX_MAIN_FUEL_VALVE_CHANNEL CHANNEL_2
using mainFuel = miosix::interfaces::timers::tim8ch2;

#define MIOSIX_PRZ_OX_VALVE_TIM TIM1
#define MIOSIX_PRZ_OX_VALVE_CHANNEL CHANNEL_1
using przOx = miosix::interfaces::timers::tim1ch1;

#define MIOSIX_PRZ_FUEL_VALVE_TIM TIM4
#define MIOSIX_PRZ_FUEL_VALVE_CHANNEL CHANNEL_1
using przFuel = miosix::interfaces::timers::tim4ch1;

#define MIOSIX_IGNITER_TIM TIM12
#define MIOSIX_IGNITER_CHANNEL CHANNEL_1
using igniter = miosix::interfaces::timers::tim4ch1;

}  // namespace servos

namespace solenoidal
{
using igniterOx         = Gpio<GPIOG_BASE, 12>;
using igniterFuelSense  = Gpio<GPIOG_BASE, 13>;
using igniterExtraSense = Gpio<GPIOG_BASE, 14>;
}  // namespace solenoidal

namespace gpios
{
using debugLedRed    = Gpio<GPIOC_BASE, 5>;
using debugLedOrange = Gpio<GPIOB_BASE, 15>;
using debugLedYellow = Gpio<GPIOD_BASE, 11>;
using debugLedGreen  = Gpio<GPIOG_BASE, 3>;
}  // namespace gpios

}  // namespace miosix
