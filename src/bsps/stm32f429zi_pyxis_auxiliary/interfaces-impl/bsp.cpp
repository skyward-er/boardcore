/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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
