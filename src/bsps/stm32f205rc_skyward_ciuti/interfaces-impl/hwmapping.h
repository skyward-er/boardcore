/***************************************************************************
 *   Copyright (C) 2016 by Silvano Seva for Skyward Experimental           *
 *   Rocketry                                                              *
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

#ifndef HWMAPPING_H
#define HWMAPPING_H

#include "interfaces/gpio.h"

namespace miosix
{

namespace interfaces
{

namespace spi1
{
using cs   = Gpio<GPIOA_BASE, 4>;
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

// LIS331HH and AD8403
namespace spi2
{
using sck  = Gpio<GPIOB_BASE, 13>;
using miso = Gpio<GPIOB_BASE, 14>;
using mosi = Gpio<GPIOB_BASE, 15>;
}  // namespace spi2

// Debug
namespace usart1
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace usart1

// Pogo pin
namespace usart2
{
using tx = Gpio<GPIOA_BASE, 2>;
using rx = Gpio<GPIOA_BASE, 3>;
}  // namespace usart2

// Pogo pin
namespace usart3
{
using tx = Gpio<GPIOB_BASE, 10>;
using rx = Gpio<GPIOB_BASE, 11>;
}  // namespace usart3

namespace can1
{
using rx = Gpio<GPIOA_BASE, 11>;
using tx = Gpio<GPIOA_BASE, 12>;
}  // namespace can1

}  // namespace interfaces

namespace devices
{

namespace lis331hh
{
using cs = Gpio<GPIOC_BASE, 4>;
}

namespace ad5204
{
using cs = Gpio<GPIOC_BASE, 0>;
}

namespace ina188
{
using vsense1 = Gpio<GPIOA_BASE, 0>;
using vsense2 = Gpio<GPIOA_BASE, 1>;
using mosfet1 = Gpio<GPIOC_BASE, 3>;
using mosfet2 = Gpio<GPIOC_BASE, 2>;
}  // namespace ina188

using vbat = Gpio<GPIOC_BASE, 5>;

namespace buttons
{
using bypass = Gpio<GPIOC_BASE, 7>;
using record = Gpio<GPIOA_BASE, 8>;
}  // namespace buttons

namespace buzzer
{
using drive = Gpio<GPIOC_BASE, 6>;  // PWM TIM8_CH1
}

namespace leds
{
using led1 = Gpio<GPIOB_BASE, 0>;  // TIM3_CH3
using led2 = Gpio<GPIOB_BASE, 5>;
using led3 = Gpio<GPIOB_BASE, 6>;
using led4 = Gpio<GPIOB_BASE, 7>;
}  // namespace leds

}  // namespace devices

}  // namespace miosix

#endif  // HWMAPPING_H
