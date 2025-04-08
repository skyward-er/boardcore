/* Copyright (c) 2025 Skyward Experimental Rocketry
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
#define MIOSIX_SENSOR_LSM6DSRX_0_SPI SPI3
#define MIOSIX_SENSOR_LSM6DSRX_1_SPI SPI3
#define MIOSIX_SENSOR_LIS2MDL_EXT_SPI SPI3
#define MIOSIX_SENSOR_LIS2MDL_SPI SPI3
#define MIOSIX_SENSOR_UBXGPS_SPI SPI3
#define MIOSIX_SENSOR_ADS131M08_SPI SPI4
#define MIOSIX_SENSOR_ND015A_0_SPI SPI4
#define MIOSIX_SENSOR_ND015A_1_SPI SPI4
#define MIOSIX_SENSOR_ND015A_2_SPI SPI4
#define MIOSIX_SENSOR_ND015A_3_SPI SPI4

#define MIOSIX_SENSOR_VN100_SYNC_IN EXTI7_IRQHandlerImpl
#define MIOSIX_SENSOR_VN100_SPI SPI1

#define MIOSIX_RADIO_DIO0_IRQ EXTI11_IRQHandlerImpl
#define MIOSIX_RADIO_DIO1_IRQ EXTI3_IRQHandlerImpl
#define MIOSIX_RADIO_DIO3_IRQ EXTI6_IRQHandlerImpl
#define MIOSIX_RADIO_SPI SPI6

#define MIOSIX_SERVOS_1_TIM TIM4
#define MIOSIX_SERVOS_1_CHANNEL CHANNEL_2
#define MIOSIX_SERVOS_2_TIM TIM3
#define MIOSIX_SERVOS_2_CHANNEL CHANNEL_1
#define MIOSIX_SERVOS_3_TIM TIM9
#define MIOSIX_SERVOS_3_CHANNEL CHANNEL_1
#define MIOSIX_BUZZER_TIM TIM1
#define MIOSIX_BUZZER_CHANNEL CHANNEL_1

#define MIOSIX_PARAFOIL_SERVO_1_TIM MIOSIX_SERVOS_1_TIM
#define MIOSIX_PARAFOIL_SERVO_1_CHANNEL MIOSIX_SERVOS_1_CHANNEL
#define MIOSIX_PARAFOIL_SERVO_2_TIM MIOSIX_SERVOS_2_TIM
#define MIOSIX_PARAFOIL_SERVO_2_CHANNEL MIOSIX_SERVOS_2_CHANNEL
#define MIOSIX_AIRBRAKES_TIM MIOSIX_SERVOS_2_TIM
#define MIOSIX_AIRBRAKES_CHANNEL MIOSIX_SERVOS_2_CHANNEL
#define MIOSIX_EXPULSION_TIM MIOSIX_SERVOS_3_TIM
#define MIOSIX_EXPULSION_CHANNEL MIOSIX_SERVOS_3_CHANNEL

namespace miosix
{

namespace interfaces
{

// H3LIS, LPS22, VN100
namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

// LSM6_0, LSM6_1, LIS2MDL, LIS2MDL_EXT, GPS
namespace spi3
{
using sck  = Gpio<GPIOB_BASE, 3>;
using miso = Gpio<GPIOB_BASE, 4>;
using mosi = Gpio<GPIOD_BASE, 6>;
}  // namespace spi3

// ADS131, ND015A_0, NDX015A_1, NDX015A_2, NDX015A_3
namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

// Radio
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
using tim4ch2 = Gpio<GPIOD_BASE, 13>;
using tim9ch1 = Gpio<GPIOA_BASE, 2>;
}  // namespace timers

namespace adcs
{
using adc12in8 = Gpio<GPIOB_BASE, 0>;
using adc12in9 = Gpio<GPIOB_BASE, 1>;
}  // namespace adcs

}  // namespace interfaces

namespace sensors
{

namespace ADS131M08
{
using cs = Gpio<GPIOG_BASE, 10>;
}

// Either the LPS22DF or the LIS2MDL_EXTERNAL sensors could be attached to the
// PD7 pin (not together)
namespace LPS22DF
{
using cs = Gpio<GPIOD_BASE, 7>;
}

namespace LIS2MDL_EXT
{
using cs = Gpio<GPIOD_BASE, 7>;
}

// The PG11 pin is either attached to the internal or the external LIS2MDL, it
// doesn't matter software-side since they are the same sensor
namespace LIS2MDL
{
using cs = Gpio<GPIOG_BASE, 11>;
}

namespace H3LIS331DL
{
using cs = Gpio<GPIOG_BASE, 9>;
}

namespace LSM6DSRX_0
{
using cs = Gpio<GPIOD_BASE, 12>;
}

namespace LSM6DSRX_1
{
using cs = Gpio<GPIOC_BASE, 7>;
}

namespace UBXGps
{
using cs = Gpio<GPIOG_BASE, 7>;
}

namespace VN100
{
using cs      = Gpio<GPIOA_BASE, 15>;
using syncIn  = Gpio<GPIOB_BASE, 7>;
using syncOut = Gpio<GPIOE_BASE, 3>;
}  // namespace VN100

namespace ND015A_0
{
using cs = Gpio<GPIOD_BASE, 5>;
}

namespace ND015A_1
{
using cs = Gpio<GPIOB_BASE, 9>;
}

namespace ND015A_2
{
using cs = Gpio<GPIOB_BASE, 8>;
}

namespace ND015A_3
{
using cs = Gpio<GPIOD_BASE, 3>;
}

}  // namespace sensors

namespace radio
{
using cs   = Gpio<GPIOC_BASE, 4>;
using txEn = Gpio<GPIOC_BASE, 5>;
using rxEn = Gpio<GPIOB_BASE, 15>;
using dio0 = Gpio<GPIOD_BASE, 11>;
using dio1 = Gpio<GPIOG_BASE, 3>;
using dio3 = Gpio<GPIOG_BASE, 6>;
using nrst = Gpio<GPIOE_BASE, 4>;
}  // namespace radio

namespace adcs
{
using vbat    = miosix::interfaces::adcs::adc12in8;
using camVbat = miosix::interfaces::adcs::adc12in9;
}  // namespace adcs

namespace sense
{
using detachPayload  = Gpio<GPIOC_BASE, 3>;
using detachMain     = Gpio<GPIOA_BASE, 4>;
using detachRamp     = Gpio<GPIOB_BASE, 11>;
using expulsionSense = Gpio<GPIOA_BASE, 3>;
using cutterSense    = Gpio<GPIOC_BASE, 1>;
}  // namespace sense

namespace gpios
{
using mainDeploy = Gpio<GPIOD_BASE, 4>;
using boardLed   = Gpio<GPIOB_BASE, 12>;
using statusLed  = Gpio<GPIOB_BASE, 13>;
using camEnable  = Gpio<GPIOB_BASE, 14>;
}  // namespace gpios

}  // namespace miosix
