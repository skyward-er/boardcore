/***************************************************************************
 *   Copyright (C) 2014 by Terraneo Federico                               *
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
#include "drivers/sd_stm32f2_f4_f7.h"
#include "drivers/serial.h"
#include "filesystem/console/console_device.h"
#include "filesystem/file_access.h"
#include "hwmapping.h"
#include "interfaces/arch_registers.h"
#include "interfaces/delays.h"
#include "interfaces/portability.h"
#include "kernel/kernel.h"
#include "kernel/logging.h"
#include "kernel/sync.h"

namespace miosix
{

void IRQbspInit()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOEEN |
                    RCC_AHB1ENR_GPIOFEN;

    // Enable can1
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    RCC_SYNC();

    using namespace interfaces;

    debug::rx::mode(Mode::ALTERNATE);
    debug::rx::alternateFunction(8);
    debug::tx::mode(Mode::ALTERNATE);
    debug::tx::alternateFunction(8);

    cam1::rx::mode(Mode::ALTERNATE);
    cam1::rx::alternateFunction(7);
    cam1::tx::mode(Mode::ALTERNATE);
    cam1::tx::alternateFunction(7);

    cam2::tx::mode(Mode::ALTERNATE);
    cam2::tx::alternateFunction(8);

    cam3::tx::mode(Mode::ALTERNATE);
    cam3::tx::alternateFunction(8);

    camMosfet::mode(Mode::OUTPUT);
    camMosfet::low();

    can1::rx::mode(Mode::ALTERNATE);
    can1::rx::alternateFunction(9);
    can1::tx::mode(Mode::ALTERNATE);
    can1::tx::alternateFunction(9);

    using namespace leds;

    led1::mode(Mode::OUTPUT);
    led2::mode(Mode::OUTPUT);
    led3::mode(Mode::OUTPUT);
    led4::mode(Mode::OUTPUT);
    led5::mode(Mode::OUTPUT);

    for (uint8_t i = 0; i < 3; i++)
    {
        ledOn();
        delayMs(10);
        ledOff();
        delayMs(10);
    }

    DefaultConsole::instance().IRQset(intrusive_ref_ptr<Device>(
        new STM32Serial(defaultSerial, defaultSerialSpeed,
                        defaultSerialFlowctrl ? STM32Serial::RTSCTS
                                              : STM32Serial::NOFLOWCTRL)));
}

void bspInit2() {}

/**
 * For safety reasons, we never want the board to shutdown.
 * When requested to shutdown, we reboot instead.
 */
void shutdown() { reboot(); }

void reboot()
{
    ioctl(STDOUT_FILENO, IOCTL_SYNC, 0);

    disableInterrupts();
    miosix_private::IRQsystemReboot();
}

}  // namespace miosix
