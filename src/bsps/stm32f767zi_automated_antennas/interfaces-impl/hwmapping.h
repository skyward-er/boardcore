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

#define MIOSIX_ETHERNET_IRQ EXTI1_IRQHandlerImpl
#define MIOSIX_ETHERNET_SPI SPI4

namespace miosix
{
namespace interfaces
{
// RADIO 1
namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

// FREE SPI
namespace spi3
{
using sck  = Gpio<GPIOB_BASE, 3>;
using miso = Gpio<GPIOB_BASE, 4>;
using mosi = Gpio<GPIOD_BASE, 6>;
}  // namespace spi3

// ETHERNET
namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

namespace can2
{
using rx = Gpio<GPIOB_BASE, 12>;
using tx = Gpio<GPIOB_BASE, 13>;
}  // namespace can2

// DBG
namespace usart1
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace usart1

// FREE USART
namespace usart2
{
using tx = Gpio<GPIOA_BASE, 2>;
using rx = Gpio<GPIOA_BASE, 3>;
}  // namespace usart2

namespace timers
{
using tim3ch2 = Gpio<GPIOC_BASE, 7>;   // step 1
using tim1ch4 = Gpio<GPIOA_BASE, 11>;  // count 1
using tim4ch1 = Gpio<GPIOD_BASE, 12>;  // step 2
using tim8ch4 = Gpio<GPIOC_BASE, 9>;   // count 2
}  // namespace timers

}  // namespace interfaces

namespace radio
{
using sck       = interfaces::spi1::sck;
using miso      = interfaces::spi1::miso;
using mosi      = interfaces::spi1::mosi;
using cs        = Gpio<GPIOA_BASE, 4>;
using dio0      = Gpio<GPIOC_BASE, 6>;
using dio1      = Gpio<GPIOD_BASE, 4>;
using dio3      = Gpio<GPIOD_BASE, 5>;
using rx_enable = Gpio<GPIOB_BASE, 9>;
using tx_enable = Gpio<GPIOB_BASE, 8>;
}  // namespace radio

namespace stepper1
{
using enable     = Gpio<GPIOA_BASE, 8>;
using direction  = Gpio<GPIOA_BASE, 12>;
using pulseTimer = interfaces::timers::tim3ch2;
using countTimer = interfaces::timers::tim1ch4;
}  // namespace stepper1

namespace stepper2
{
using enable     = Gpio<GPIOB_BASE, 14>;
using direction  = Gpio<GPIOG_BASE, 7>;
using pulseTimer = interfaces::timers::tim4ch1;
using countTimer = interfaces::timers::tim8ch4;
}  // namespace stepper2

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