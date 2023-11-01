/***************************************************************************
 *   Copyright (C) 2023 by Skyward                                         *
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

#define MIOSIX_RADIO1_DIO0_IRQ EXTI8_IRQHandlerImpl
#define MIOSIX_RADIO1_DIO1_IRQ EXTI10_IRQHandlerImpl
#define MIOSIX_RADIO1_DIO2_IRQ EXTI11_IRQHandlerImpl
#define MIOSIX_RADIO1_DIO3_IRQ EXTI12_IRQHandlerImpl
#define MIOSIX_RADIO1_SPI SPI3

#define MIOSIX_RADIO2_DIO0_IRQ EXTI6_IRQHandlerImpl
#define MIOSIX_RADIO2_DIO1_IRQ EXTI4_IRQHandlerImpl
#define MIOSIX_RADIO2_DIO2_IRQ EXTI7_IRQHandlerImpl
#define MIOSIX_RADIO2_DIO3_IRQ EXTI5_IRQHandlerImpl
#define MIOSIX_RADIO2_DIO4_IRQ EXTI2_IRQHandlerImpl
#define MIOSIX_RADIO2_DIO5_IRQ EXTI3_IRQHandlerImpl
#define MIOSIX_RADIO2_SPI SPI1

#define MIOSIX_ETHERNET_IRQ EXTI1_IRQHandlerImpl
#define MIOSIX_ETHERNET_SPI SPI4

// Remember to modify pins of leds
namespace miosix
{

namespace interfaces
{

// Radio 1
namespace spi3
{
using sck  = Gpio<GPIOB_BASE, 3>;
using miso = Gpio<GPIOB_BASE, 4>;
using mosi = Gpio<GPIOD_BASE, 6>;
}  // namespace spi3

// Radio 2
namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

// Ethernet module
namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

}  // namespace interfaces

namespace radio1
{
namespace spi
{
using namespace miosix::interfaces::spi3;
}  // namespace spi

using cs   = Gpio<GPIOA_BASE, 15>;
using dio0 = Gpio<GPIOC_BASE, 8>;
using dio1 = Gpio<GPIOC_BASE, 10>;
using dio3 = Gpio<GPIOC_BASE, 12>;
using txen = Gpio<GPIOG_BASE, 12>;
using rxen = Gpio<GPIOG_BASE, 14>;
using nrst = Gpio<GPIOA_BASE, 1>;
}  // namespace radio1

namespace radio2
{
namespace spi
{
using namespace miosix::interfaces::spi1;
}  // namespace spi

using cs   = Gpio<GPIOA_BASE, 4>;
using dio0 = Gpio<GPIOC_BASE, 6>;
using dio1 = Gpio<GPIOD_BASE, 4>;
using dio3 = Gpio<GPIOD_BASE, 5>;
using txen = Gpio<GPIOB_BASE, 8>;
using rxen = Gpio<GPIOB_BASE, 9>;
using nrst = Gpio<GPIOA_BASE, 0>;
}  // namespace radio2

namespace ethernet
{
namespace spi
{
using namespace miosix::interfaces::spi4;
}  // namespace spi

using cs   = Gpio<GPIOE_BASE, 4>;
using intr = Gpio<GPIOC_BASE, 1>;
using nrst = Gpio<GPIOB_BASE, 1>;
}  // namespace ethernet

}  // namespace miosix

#endif  // HWMAPPING_H
