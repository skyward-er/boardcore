/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Davide Mor
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
#include "drivers/dcc.h"
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
