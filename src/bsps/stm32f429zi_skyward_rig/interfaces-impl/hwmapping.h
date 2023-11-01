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
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

namespace spi2
{
using sck  = Gpio<GPIOD_BASE, 3>;
using miso = Gpio<GPIOC_BASE, 2>;
using mosi = Gpio<GPIOC_BASE, 3>;
}  // namespace spi2

namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

namespace spi5
{
using sck  = Gpio<GPIOF_BASE, 7>;
using miso = Gpio<GPIOF_BASE, 8>;
using mosi = Gpio<GPIOF_BASE, 9>;
}  // namespace spi5

namespace spi6
{
using sck  = Gpio<GPIOG_BASE, 13>;
using miso = Gpio<GPIOG_BASE, 12>;
using mosi = Gpio<GPIOG_BASE, 14>;
}  // namespace spi6

// USB UART
namespace uart1
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace uart1

namespace timers
{
using tim4ch2  = Gpio<GPIOB_BASE, 7>;  // Servo 1
using tim11ch1 = Gpio<GPIOB_BASE, 9>;  // Servo 2
using tim3ch1  = Gpio<GPIOB_BASE, 4>;  // Servo 3
using tim10ch1 = Gpio<GPIOB_BASE, 8>;  // Servo 4
using tim8ch1  = Gpio<GPIOC_BASE, 6>;  // Servo 5
}  // namespace timers

}  // namespace interfaces

namespace sensors
{

namespace ADS131_1
{
using cs   = Gpio<GPIOB_BASE, 1>;
using sck  = interfaces::spi1::sck;
using miso = interfaces::spi1::miso;
using mosi = interfaces::spi1::mosi;
}  // namespace ADS131_1

namespace ADS131_2
{
using cs   = Gpio<GPIOE_BASE, 4>;
using sck  = interfaces::spi1::sck;
using miso = interfaces::spi1::miso;
using mosi = interfaces::spi1::mosi;
}  // namespace ADS131_2

namespace MAX31855
{
using cs   = Gpio<GPIOF_BASE, 9>;
using sck  = interfaces::spi1::sck;
using miso = interfaces::spi1::miso;
using mosi = interfaces::spi1::mosi;
}  // namespace MAX31855

namespace HX711_1
{
using sck = interfaces::spi2::sck;
}
namespace HX711_2
{
using sck = interfaces::spi6::sck;
}

namespace HX711_3
{
using sck = interfaces::spi4::sck;
}

}  // namespace sensors

namespace servos
{
using servo1 = interfaces::timers::tim4ch2;
using servo2 = interfaces::timers::tim11ch1;
using servo3 = interfaces::timers::tim3ch1;
using servo4 = interfaces::timers::tim10ch1;
using servo5 = interfaces::timers::tim8ch1;
}  // namespace servos

namespace relays
{
using ignition       = Gpio<GPIOC_BASE, 14>;
using ledLamp        = Gpio<GPIOE_BASE, 3>;
using nitrogen       = Gpio<GPIOC_BASE, 13>;
using generalPurpose = Gpio<GPIOA_BASE, 15>;
}  // namespace relays

namespace radio
{
using cs   = Gpio<GPIOF_BASE, 6>;
using sck  = interfaces::spi4::sck;
using miso = interfaces::spi4::miso;
using mosi = interfaces::spi4::mosi;
using dio0 = Gpio<GPIOD_BASE, 5>;
using dio1 = Gpio<GPIOD_BASE, 12>;
using dio3 = Gpio<GPIOD_BASE, 13>;
using txEn = Gpio<GPIOG_BASE, 2>;
using rxEn = Gpio<GPIOG_BASE, 3>;
using nrst = Gpio<GPIOB_BASE, 0>;
}  // namespace radio

namespace ui
{
using button   = Gpio<GPIOA_BASE, 0>;   // User button
using greenLed = Gpio<GPIOG_BASE, 13>;  // Green LED
using redLed   = Gpio<GPIOG_BASE, 14>;  // Red LED
}  // namespace ui

}  // namespace miosix
