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

namespace miosix
{

namespace interfaces
{

// CC3135
namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOB_BASE, 4>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

// RA-01
namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

}  // namespace interfaces

namespace peripherals
{

namespace ra01
{

namespace pc13
{
using cs   = Gpio<GPIOC_BASE, 13>;
using dio0 = Gpio<GPIOF_BASE, 6>;
using dio1 = Gpio<GPIOA_BASE, 4>;
using dio3 = Gpio<GPIOC_BASE, 11>;
using nrst = Gpio<GPIOC_BASE, 14>;
}  // namespace pc13

namespace pe4
{
using cs   = Gpio<GPIOE_BASE, 4>;
using dio0 = Gpio<GPIOE_BASE, 3>;
using nrst = Gpio<GPIOG_BASE, 2>;
}  // namespace pe4

}  // namespace ra01

namespace cc3135
{
using cs   = Gpio<GPIOD_BASE, 4>;
using hib  = Gpio<GPIOG_BASE, 3>;
using intr = Gpio<GPIOD_BASE, 5>;
using nrst = Gpio<GPIOB_BASE, 7>;
}  // namespace cc3135

}  // namespace peripherals

}  // namespace miosix

#endif  // HWMAPPING_H
