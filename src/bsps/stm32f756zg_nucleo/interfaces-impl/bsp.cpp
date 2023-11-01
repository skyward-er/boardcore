/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Lorenzo Cucchi
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
#include "interfaces/arch_registers.h"
#include "interfaces/delays.h"
#include "interfaces/portability.h"
#include "kernel/kernel.h"
#include "kernel/logging.h"
#include "kernel/sync.h"

namespace miosix
{

//
// Initialization
//

void IRQbspInit()
{
    // Enable all gpios
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN |
                    RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN |
                    RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN;
    RCC_SYNC();
    GPIOA->OSPEEDR = 0xaaaaaaaa;  // Default to 50MHz speed for all GPIOS
    GPIOB->OSPEEDR = 0xaaaaaaaa;
    GPIOC->OSPEEDR = 0xaaaaaaaa;
    GPIOD->OSPEEDR = 0xaaaaaaaa;
    GPIOE->OSPEEDR = 0xaaaaaaaa;
    GPIOF->OSPEEDR = 0xaaaaaaaa;
    GPIOG->OSPEEDR = 0xaaaaaaaa;
    GPIOH->OSPEEDR = 0xaaaaaaaa;

    userLed1::mode(Mode::OUTPUT);
    userLed2::mode(Mode::OUTPUT);
    userLed3::mode(Mode::OUTPUT);
    userBtn::mode(Mode::INPUT);

    ledOn();
    delayMs(100);
    ledOff();
    auto tx = Gpio<GPIOD_BASE, 8>::getPin();
    tx.alternateFunction(7);
    auto rx = Gpio<GPIOD_BASE, 9>::getPin();
    rx.alternateFunction(7);
    DefaultConsole::instance().IRQset(intrusive_ref_ptr<Device>(
        new STM32Serial(3, defaultSerialSpeed, tx, rx)));
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

void shutdown()
{
    ioctl(STDOUT_FILENO, IOCTL_SYNC, 0);

#ifdef WITH_FILESYSTEM
    FilesystemManager::instance().umountAll();
#endif  // WITH_FILESYSTEM

    disableInterrupts();
    for (;;)
        ;
}

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
