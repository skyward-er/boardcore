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

#define MIOSIX_SENSOR_H3LIS_SPI SPI4
#define MIOSIX_SENSOR_LPS22_SPI SPI4
#define MIOSIX_SENSOR_LSM6_SPI SPI4
#define MIOSIX_SENSOR_LIS2MDL_SPI SPI4
#define MIOSIX_SENSOR_ADS131_1_SPI SPI1
#define MIOSIX_SENSOR_ADS131_2_SPI SPI3
#define MIOSIX_SENSOR_MAX31856_1_SPI SPI3
#define MIOSIX_SENSOR_MAX31856_2_SPI SPI1

#define MIOSIX_RADIO_DIO0_IRQ EXTI3_IRQHandlerImpl
#define MIOSIX_RADIO_DIO1_IRQ EXTI4_IRQHandlerImpl
#define MIOSIX_RADIO_DIO3_IRQ EXTI9_IRQHandlerImpl
#define MIOSIX_RADIO_SPI SPI6

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

namespace spi2
{
using sck  = Gpio<GPIOD_BASE, 3>;
using miso = Gpio<GPIOB_BASE, 14>;
using mosi = Gpio<GPIOC_BASE, 3>;
}  // namespace spi2

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

// Miosix UART
namespace usart1
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace usart1

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

namespace can2
{
using tx = Gpio<GPIOB_BASE, 13>;
using rx = Gpio<GPIOB_BASE, 12>;
}  // namespace can2

namespace timers
{
using tim1ch1  = Gpio<GPIOA_BASE, 8>;
using tim2ch3  = Gpio<GPIOA_BASE, 2>;
using tim2ch4  = Gpio<GPIOB_BASE, 11>;
using tim3ch3  = Gpio<GPIOB_BASE, 0>;
using tim4ch1  = Gpio<GPIOD_BASE, 12>;
using tim4ch2  = Gpio<GPIOD_BASE, 13>;
using tim8ch1  = Gpio<GPIOC_BASE, 6>;
using tim8ch2  = Gpio<GPIOC_BASE, 7>;
using tim9ch2  = Gpio<GPIOA_BASE, 3>;
using tim10ch1 = Gpio<GPIOB_BASE, 8>;
using tim11ch1 = Gpio<GPIOB_BASE, 9>;
using tim12ch2 = Gpio<GPIOB_BASE, 15>;
}  // namespace timers

namespace adcs
{
using adc12in9   = Gpio<GPIOB_BASE, 1>;
using adc123in11 = Gpio<GPIOC_BASE, 1>;
using adc12in14  = Gpio<GPIOC_BASE, 4>;
}  // namespace adcs

namespace relays
{
using relay1 = Gpio<GPIOA_BASE, 15>;
using relay2 = Gpio<GPIOG_BASE, 3>;
using relay3 = Gpio<GPIOG_BASE, 6>;
using relay4 = Gpio<GPIOG_BASE, 7>;
}  // namespace relays

}  // namespace interfaces

namespace sensors
{
// Flavio, why?
namespace H3LIS
{
using cs = Gpio<GPIOB_BASE, 2>;
}  // namespace H3LIS

// Flavio, why?
namespace LPS22
{
// NOTE: also used by tim2ch4, ensure there's no conflict before enabling this
// using cs = Gpio<GPIOB_BASE, 11>;
}  // namespace LPS22

// Flavio, why?
namespace LSM6
{
// NOTE: also used by tim4ch2, ensure there's no conflict before enabling this
// using cs = Gpio<GPIOD_BASE, 13>;
}  // namespace LSM6

// Flavio, why?
namespace LIS2MDL
{
using cs = Gpio<GPIOA_BASE, 4>;
}  // namespace LIS2MDL

namespace ADS131_1
{
using cs = Gpio<GPIOC_BASE, 5>;
}  // namespace ADS131_1

namespace ADS131_2
{
using cs = Gpio<GPIOD_BASE, 4>;
}  // namespace ADS131_2

namespace MAX31856_1
{
using cs = Gpio<GPIOG_BASE, 10>;
}  // namespace MAX31856_1

namespace MAX31856_2
{
using cs = Gpio<GPIOD_BASE, 7>;
}  // namespace MAX31856_2

}  // namespace sensors

namespace radio
{
using cs   = Gpio<GPIOB_BASE, 7>;
using txEn = Gpio<GPIOG_BASE, 11>;
using rxEn = Gpio<GPIOD_BASE, 5>;
using dio0 = Gpio<GPIOE_BASE, 3>;
using dio1 = Gpio<GPIOE_BASE, 4>;
using dio3 = Gpio<GPIOG_BASE, 9>;
}  // namespace radio

/**
 * @brief Servo PWM control PIN definitions
 *
 * @note Some servos share the same timer, so they must use the same frequency:
 * - Servo OX_FIL  and PRZ_OX    (TIM8) must use the same frequency
 * - Servo PRZ_REL and FUEL_VENT (TIM2) must use the same frequency
 * - Servo PRZ_FIL and MAIN_OX   (TIM4) must use the same frequency
 */
namespace servos
{
#define MIOSIX_SERVOS_OX_FIL_TIM TIM8
#define MIOSIX_SERVOS_OX_FIL_CHANNEL CHANNEL_2
using servo_ox_filling = miosix::interfaces::timers::tim8ch2;

#define MIOSIX_SERVOS_OX_REL_TIM TIM1
#define MIOSIX_SERVOS_OX_REL_CHANNEL CHANNEL_1
using servo_ox_release = miosix::interfaces::timers::tim1ch1;

#define MIOSIX_SERVOS_PRZ_FUEL_TIM TIM10  // changed ox detach -> prz fuel
#define MIOSIX_SERVOS_PRZ_FUEL_CHANNEL CHANNEL_1
using servo_prz_fuel = miosix::interfaces::timers::tim10ch1;

#define MIOSIX_SERVOS_PRZ_3W_TIM TIM11
#define MIOSIX_SERVOS_PRZ_3W_CHANNEL CHANNEL_1
using servo_prz_3way = miosix::interfaces::timers::tim11ch1;

#define MIOSIX_SERVOS_PRZ_FIL_TIM TIM4
#define MIOSIX_SERVOS_PRZ_FIL_CHANNEL CHANNEL_1
using servo_prz_filling = miosix::interfaces::timers::tim4ch1;

#define MIOSIX_SERVOS_PRZ_REL_TIM TIM2
#define MIOSIX_SERVOS_PRZ_REL_CHANNEL CHANNEL_3
using servo_prz_release = miosix::interfaces::timers::tim2ch3;

#define MIOSIX_SERVOS_MAIN_FUEL_TIM TIM3  // changed prz det -> main fuel
#define MIOSIX_SERVOS_MAIN_FUEL_CHANNEL CHANNEL_3
using servo_main_fuel = miosix::interfaces::timers::tim3ch3;

#define MIOSIX_SERVOS_PRZ_OX_TIM TIM8  // changed nitrogen -> prz ox
#define MIOSIX_SERVOS_PRZ_OX_CHANNEL CHANNEL_1
using servo_prz_ox = miosix::interfaces::timers::tim8ch1;

#define MIOSIX_SERVOS_OX_VEN_TIM TIM12
#define MIOSIX_SERVOS_OX_VEN_CHANNEL CHANNEL_2
using servo_ox_venting = miosix::interfaces::timers::tim12ch2;

#define MIOSIX_SERVOS_PRZ_QUE_TIM TIM9
#define MIOSIX_SERVOS_PRZ_QUE_CHANNEL CHANNEL_2
using servo_PRZ_quenching = miosix::interfaces::timers::tim9ch2;

#define MIOSIX_SERVOS_MAIN_OX_TIM TIM4  // changed main -> main OX
#define MIOSIX_SERVOS_MAIN_OX_CHANNEL CHANNEL_2
using servo_main_ox = miosix::interfaces::timers::tim4ch2;

#define MIOSIX_SERVOS_FUEL_VEN_TIM TIM2  // changed unused -> fuel vent
#define MIOSIX_SERVOS_FUEL_VEN_CHANNEL CHANNEL_4
using servo_fuel_ven = miosix::interfaces::timers::tim2ch4;
}  // namespace servos

namespace adcs
{
using umbilicalCur = miosix::interfaces::adcs::adc123in11;
using servoCur     = miosix::interfaces::adcs::adc12in9;
using batteryVolt  = miosix::interfaces::adcs::adc12in14;
}  // namespace adcs

namespace relays
{
using clacson  = miosix::interfaces::relays::relay1;
using ignition = miosix::interfaces::relays::relay2;
using nitrogen = miosix::interfaces::relays::relay3;
using armLight = miosix::interfaces::relays::relay4;
}  // namespace relays

}  // namespace miosix
