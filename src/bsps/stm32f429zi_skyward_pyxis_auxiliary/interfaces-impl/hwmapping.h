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

#ifndef HWMAPPING_H
#define HWMAPPING_H

#include "interfaces/gpio.h"

// Remember to modify pins of leds
namespace miosix
{

namespace interfaces
{

// Debug - USART4
namespace debug
{
using tx = Gpio<GPIOA_BASE, 0>;
using rx = Gpio<GPIOA_BASE, 1>;
}  // namespace debug

// Cam 1 - USART2
namespace cam1
{
using tx = Gpio<GPIOA_BASE, 2>;
using rx = Gpio<GPIOA_BASE, 3>;
}  // namespace cam1

// Cam 2 - UART7
namespace cam2
{
using tx = Gpio<GPIOF_BASE, 7>;
}  // namespace cam2

// Cam 3 - UART7
namespace cam3
{
using tx = Gpio<GPIOE_BASE, 8>;
}  // namespace cam3

using camMosfet = Gpio<GPIOE_BASE, 7>;

namespace can1
{
using rx = Gpio<GPIOA_BASE, 11>;
using tx = Gpio<GPIOA_BASE, 12>;
}  // namespace can1

}  // namespace interfaces

namespace leds
{
using led1 = Gpio<GPIOC_BASE, 15>;
using led2 = Gpio<GPIOB_BASE, 4>;
using led3 = Gpio<GPIOB_BASE, 5>;
using led4 = Gpio<GPIOB_BASE, 6>;
using led5 = Gpio<GPIOB_BASE, 7>;
}  // namespace leds

}  // namespace miosix

#endif  // HWMAPPING_H
