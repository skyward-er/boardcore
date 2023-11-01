/***************************************************************************
 *   Copyright (C) 2016 by Terraneo Federico and Silvano Seva for          *
 *   Skyward Experimental Rocketry                                         *
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

/***********************************************************************
 * bsp.cpp Part of the Miosix Embedded OS.
 * Board support package, this file initializes hardware.
 ************************************************************************/

#include "interfaces/bsp.h"

#include <inttypes.h>
#include <sys/ioctl.h>

#include <cstdlib>

#include "board_settings.h"
#include "config/miosix_settings.h"
#include "drivers/dcc.h"
#include "drivers/serial.h"
#include "filesystem/console/console_device.h"
#include "filesystem/file_access.h"
#include "interfaces/arch_registers.h"
#include "interfaces/delays.h"
#include "interfaces/portability.h"
#include "kernel/kernel.h"
#include "kernel/logging.h"
#include "kernel/sync.h"

using namespace std;

namespace miosix
{

//
// Initialization
//

void IRQbspInit()
{
    // Enable all gpios
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
    RCC_SYNC();
    GPIOA->OSPEEDR = 0xaaaaaaaa;  // Default to 50MHz speed for all GPIOS
    GPIOB->OSPEEDR = 0xaaaaaaaa;
    GPIOC->OSPEEDR = 0xaaaaaaaa;
    GPIOD->OSPEEDR = 0xaaaaaaaa;

    using namespace interfaces;

    spi1::cs::mode(Mode::OUTPUT);
    spi1::cs::high();
    spi1::sck::mode(Mode::ALTERNATE);
    spi1::sck::alternateFunction(5);
    spi1::miso::mode(Mode::ALTERNATE);
    spi1::miso::alternateFunction(5);
    spi1::mosi::mode(Mode::ALTERNATE);
    spi1::mosi::alternateFunction(5);

    spi2::sck::mode(Mode::ALTERNATE);
    spi2::sck::alternateFunction(5);
    spi2::miso::mode(Mode::ALTERNATE);
    spi2::miso::alternateFunction(5);
    spi2::mosi::mode(Mode::ALTERNATE);
    spi2::mosi::alternateFunction(5);

    usart1::rx::mode(Mode::ALTERNATE);
    usart1::rx::alternateFunction(7);
    usart1::tx::mode(Mode::ALTERNATE);
    usart1::tx::alternateFunction(7);

    usart2::rx::mode(Mode::ALTERNATE);
    usart2::rx::alternateFunction(7);
    usart2::tx::mode(Mode::ALTERNATE);
    usart2::tx::alternateFunction(7);

    usart3::rx::mode(Mode::ALTERNATE);
    usart3::rx::alternateFunction(7);
    usart3::tx::mode(Mode::ALTERNATE);
    usart3::tx::alternateFunction(7);

    can1::rx::mode(Mode::ALTERNATE);
    can1::rx::alternateFunction(9);
    can1::tx::mode(Mode::ALTERNATE);
    can1::tx::alternateFunction(9);

    using namespace devices;

    lis331hh::cs::mode(Mode::OUTPUT);
    lis331hh::cs::high();

    ad5204::cs::mode(Mode::OUTPUT);
    ad5204::cs::high();

    ina188::vsense1::mode(Mode::INPUT_ANALOG);
    ina188::vsense2::mode(Mode::INPUT_ANALOG);
    ina188::mosfet1::mode(Mode::OUTPUT);
    ina188::mosfet1::high();
    ina188::mosfet2::mode(Mode::OUTPUT);
    ina188::mosfet2::high();

    vbat::mode(Mode::INPUT_ANALOG);

    buttons::bypass::mode(Mode::INPUT);
    buttons::record::mode(Mode::INPUT);

    buzzer::drive::mode(Mode::ALTERNATE);
    buzzer::drive::alternateFunction(3);

    leds::led1::mode(Mode::OUTPUT);
    leds::led1::low();
    leds::led2::mode(Mode::OUTPUT);
    leds::led2::low();
    leds::led3::mode(Mode::OUTPUT);
    leds::led3::low();
    leds::led4::mode(Mode::OUTPUT);
    leds::led4::low();

    DefaultConsole::instance().IRQset(intrusive_ref_ptr<Device>(
        new STM32Serial(defaultSerial, defaultSerialSpeed,
                        defaultSerialFlowctrl ? STM32Serial::RTSCTS
                                              : STM32Serial::NOFLOWCTRL)));
}

void bspInit2()
{
#ifdef WITH_FILESYSTEM
    basicFilesystemSetup(SDIODriver::instance());
#endif  // WITH_FILESYSTEM
}

//
// Shutdown and reboot
//

/**
 * For safety reasons, we never want to shutdown. When requested to shutdown, we
 * reboot instead.
 */
void shutdown() { reboot(); }

void reboot()
{
    ioctl(STDOUT_FILENO, IOCTL_SYNC, 0);

#ifdef WITH_FILESYSTEM
    FilesystemManager::instance().umountAll();
#endif  // WITH_FILESYSTEM

    disableInterrupts();
    miosix_private::IRQsystemReboot();
}
}  // namespace miosix
