/***************************************************************************
 *   Copyright (C) 2018 by Terraneo Federico                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

#pragma once

#include "interfaces/gpio.h"

namespace miosix
{

namespace interfaces
{

namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOB_BASE, 4>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

// USB UART
namespace uart1
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace uart1

// GPS UART
namespace uart2
{
using tx = Gpio<GPIOA_BASE, 2>;
using rx = Gpio<GPIOA_BASE, 3>;
}  // namespace uart2

namespace timers
{
using tim4ch2  = Gpio<GPIOB_BASE, 7>;  // Servo 1
using tim10ch1 = Gpio<GPIOF_BASE, 6>;  // Servo 2
}  // namespace timers

}  // namespace interfaces

namespace sensors
{

namespace mpu9250
{
using cs   = Gpio<GPIOB_BASE, 2>;
using sck  = interfaces::spi1::sck;
using miso = interfaces::spi1::miso;
using mosi = interfaces::spi1::mosi;
}  // namespace mpu9250

namespace bme280
{
using cs   = Gpio<GPIOC_BASE, 11>;
using sck  = interfaces::spi1::sck;
using miso = interfaces::spi1::miso;
using mosi = interfaces::spi1::mosi;
}  // namespace bme280

namespace gps
{
using tx = interfaces::uart2::tx;
using rx = interfaces::uart2::rx;
}  // namespace gps

namespace adc
{
using battery = Gpio<GPIOC_BASE, 3>;
}

}  // namespace sensors

namespace sx1278
{
using cs        = Gpio<GPIOC_BASE, 1>;
using interrupt = Gpio<GPIOF_BASE, 10>;
using sck       = interfaces::spi4::sck;
using miso      = interfaces::spi4::miso;
using mosi      = interfaces::spi4::mosi;
}  // namespace sx1278

namespace servos
{
using servo1 = interfaces::timers::tim4ch2;
using servo2 = interfaces::timers::tim10ch1;
}  // namespace servos

namespace ui
{
using button   = Gpio<GPIOA_BASE, 0>;   // User button
using greenLed = Gpio<GPIOG_BASE, 13>;  // Green LED
using redLed   = Gpio<GPIOG_BASE, 14>;  // Red LED
}  // namespace ui

}  // namespace miosix
