/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Niccolò Betto, Pietro Bortolus
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

#define MIOSIX_SENSOR_ADS131_0_SPI SPI2
#define MIOSIX_SENSOR_ADS131_1_SPI SPI2
#define MIOSIX_SENSOR_ADS131_2_SPI SPI2
#define MIOSIX_SENSOR_ADS131_3_SPI SPI2

#define MIOSIX_SERVO_EXPANDER_I2C I2C1
#define MIOSIX_SERVO_EXPANDER_0_ADDR 0b1001100
#define MIOSIX_SERVO_EXPANDER_1_ADDR 0b1001011

#define MIOSIX_EXPANDER_SPI SPI3
#define MIOSIX_EXPANDER_INT_A_IRQ EXTI15_IRQHandlerImpl

#define MIOSIX_RADIO_SPI SPI4
#define MIOSIX_RADIO_DIO0_IRQ EXTI3_IRQHandlerImpl
#define MIOSIX_RADIO_DIO1_IRQ EXTI4_IRQHandlerImpl
#define MIOSIX_RADIO_DIO2_IRQ EXTI7_IRQHandlerImpl
#define MIOSIX_RADIO_DIO3_IRQ EXTI8_IRQHandlerImpl
#define MIOSIX_RADIO_DIO4_IRQ EXTI9_IRQHandlerImpl
#define MIOSIX_RADIO_DIO5_IRQ EXTI10_IRQHandlerImpl

namespace miosix
{
namespace interfaces
{
namespace spi2
{
using sck  = Gpio<GPIOD_BASE, 3>;
using miso = Gpio<GPIOB_BASE, 14>;
using mosi = Gpio<GPIOC_BASE, 3>;

using cs1 = Gpio<GPIOB_BASE, 2>;
using cs2 = Gpio<GPIOD_BASE, 4>;
using cs3 = Gpio<GPIOD_BASE, 5>;
using cs4 = Gpio<GPIOB_BASE, 13>;
using cs5 = Gpio<GPIOD_BASE, 12>;
}  // namespace spi2

namespace spi3
{
using sck  = Gpio<GPIOB_BASE, 3>;
using miso = Gpio<GPIOB_BASE, 4>;
using mosi = Gpio<GPIOD_BASE, 6>;

using cs6  = Gpio<GPIOD_BASE, 13>;
using cs7  = Gpio<GPIOC_BASE, 6>;
using cs8  = Gpio<GPIOC_BASE, 7>;
using cs9  = Gpio<GPIOG_BASE, 3>;
using cs10 = Gpio<GPIOG_BASE, 6>;
}  // namespace spi3

namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

// Servo Expander
namespace i2c1
{
using sda = Gpio<GPIOB_BASE, 9>;
using scl = Gpio<GPIOB_BASE, 8>;
}  // namespace i2c1

// Miosix UART
namespace usart1
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace usart1

namespace can1
{
using tx = Gpio<GPIOA_BASE, 12>;
using rx = Gpio<GPIOA_BASE, 11>;
}  // namespace can1

namespace timers
{
using tim3ch1 = Gpio<GPIOA_BASE, 6>;
}  // namespace timers

namespace adcs
{
using adc123in3 = Gpio<GPIOA_BASE, 3>;
using adc12in8  = Gpio<GPIOB_BASE, 0>;
using adc12in9  = Gpio<GPIOB_BASE, 1>;
}  // namespace adcs
}  // namespace interfaces

namespace expander
{
using cs = Gpio<GPIOA_BASE, 0>;

using intA = Gpio<GPIOB_BASE, 15>;
}  // namespace expander

namespace radio
{
using cs   = Gpio<GPIOG_BASE, 7>;
using txEn = Gpio<GPIOG_BASE, 12>;
using rxEn = Gpio<GPIOB_BASE, 7>;

using dio0 = Gpio<GPIOE_BASE, 3>;
using dio1 = Gpio<GPIOE_BASE, 4>;
using dio2 = Gpio<GPIOD_BASE, 7>;
using dio3 = Gpio<GPIOA_BASE, 8>;
using dio4 = Gpio<GPIOG_BASE, 9>;
using dio5 = Gpio<GPIOG_BASE, 10>;
}  // namespace radio

namespace actuators
{
using oxSolenoid   = Gpio<GPIOA_BASE, 15>;
using fuelSolenoid = Gpio<GPIOA_BASE, 5>;
}  // namespace actuators

/**
 * @brief PWM control PIN definitions
 */
namespace pwm
{
#define MIOSIX_IGNITER_TIM TIM3
#define MIOSIX_IGNITER_CHANNEL CHANNEL_1
using igniter = miosix::interfaces::timers::tim3ch1;
}  // namespace pwm

namespace adcs
{
using servoCurrent1 = miosix::interfaces::adcs::adc123in3;
using servoCurrent2 = miosix::interfaces::adcs::adc12in8;
using servoCurrent3 = miosix::interfaces::adcs::adc12in9;
}  // namespace adcs
}  // namespace miosix
