/* Copyright (c) 2022 Skyward Experimental Rocketry
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
// #include "kernel/IRQDisplayPrint.h"

namespace miosix
{

//
// Initialization
//

/**
 * The example code from ST checks for the busy flag after each command.
 * Interestingly I couldn't find any mention of this in the datsheet.
 */
static void sdramCommandWait()
{
    for (int i = 0; i < 0xffff; i++)
        if ((FMC_Bank5_6->SDSR & FMC_SDSR_BUSY) == 0)
            return;
}

void configureSdram()
{
    // Enable all gpios, passing clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN |
                    RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN |
                    RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN;
    RCC_SYNC();

    // First, configure SDRAM GPIOs
    GPIOB->AFR[0] = 0x0cc00000;
    GPIOC->AFR[0] = 0x0000000c;
    GPIOD->AFR[0] = 0x000000cc;
    GPIOD->AFR[1] = 0xcc000ccc;
    GPIOE->AFR[0] = 0xc00000cc;
    GPIOE->AFR[1] = 0xcccccccc;
    GPIOF->AFR[0] = 0x00cccccc;
    GPIOF->AFR[1] = 0xccccc000;
    GPIOG->AFR[0] = 0x00cc00cc;
    GPIOG->AFR[1] = 0xc000000c;

    GPIOB->MODER = 0x00002800;
    GPIOC->MODER = 0x00000002;
    GPIOD->MODER = 0xa02a000a;
    GPIOE->MODER = 0xaaaa800a;
    GPIOF->MODER = 0xaa800aaa;
    GPIOG->MODER = 0x80020a0a;

    GPIOA->OSPEEDR = 0xaaaaaaaa;  // Default to 50MHz speed for all GPIOs...
    GPIOB->OSPEEDR =
        0xaaaaaaaa | 0x00003c00;  //...but 100MHz for the SDRAM pins
    GPIOC->OSPEEDR = 0xaaaaaaaa | 0x00000003;
    GPIOD->OSPEEDR = 0xaaaaaaaa | 0xf03f000f;
    GPIOE->OSPEEDR = 0xaaaaaaaa | 0xffffc00f;
    GPIOF->OSPEEDR = 0xaaaaaaaa | 0xffc00fff;
    GPIOG->OSPEEDR = 0xaaaaaaaa | 0xc0030f0f;
    GPIOH->OSPEEDR = 0xaaaaaaaa;

    // Since we'we un-configured PB3/PB4 from the default at boot TDO,NTRST,
    // finish the job and remove the default pull-up
    GPIOB->PUPDR = 0;

    // Second, actually start the SDRAM controller
    RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;
    RCC_SYNC();

    // SDRAM is a IS42S16400J -7 speed grade, connected to bank 2 (0xd0000000)
    // Some bits in SDCR[1] are don't care, and the have to be set in SDCR[0],
    // they aren't just don't care, the controller will fail if they aren't at 0
    FMC_Bank5_6->SDCR[0] = FMC_SDCR1_SDCLK_1  // SDRAM runs @ half CPU frequency
                           | FMC_SDCR1_RBURST  // Enable read burst
                           | 0;              //  0 delay between reads after CAS
    FMC_Bank5_6->SDCR[1] = 0                 //  8 bit column address
                           | FMC_SDCR1_NR_0  // 12 bit row address
                           | FMC_SDCR1_MWID_0  // 16 bit data bus
                           | FMC_SDCR1_NB      //  4 banks
                           |
                           FMC_SDCR1_CAS_1;  //  2 cycle CAS latency (F<133MHz)

#ifdef SYSCLK_FREQ_180MHz
    // One SDRAM clock cycle is 11.1ns
    // Some bits in SDTR[1] are don't care, and the have to be set in SDTR[0],
    // they aren't just don't care, the controller will fail if they aren't at 0
    FMC_Bank5_6->SDTR[0] = (6 - 1) << 12     // 6 cycle TRC  (66.6ns>63ns)
                           | (2 - 1) << 20;  // 2 cycle TRP  (22.2ns>15ns)
    FMC_Bank5_6->SDTR[1] = (2 - 1) << 0      // 2 cycle TMRD
                           | (7 - 1) << 4    // 7 cycle TXSR (77.7ns>70ns)
                           | (4 - 1) << 8    // 4 cycle TRAS (44.4ns>42ns)
                           | (2 - 1) << 16   // 2 cycle TWR
                           | (2 - 1) << 24;  // 2 cycle TRCD (22.2ns>15ns)
#elif defined(SYSCLK_FREQ_168MHz)
    // One SDRAM clock cycle is 11.9ns
    // Some bits in SDTR[1] are don't care, and the have to be set in SDTR[0],
    // they aren't just don't care, the controller will fail if they aren't at 0
    FMC_Bank5_6->SDTR[0] = (6 - 1) << 12     // 6 cycle TRC  (71.4ns>63ns)
                           | (2 - 1) << 20;  // 2 cycle TRP  (23.8ns>15ns)
    FMC_Bank5_6->SDTR[1] = (2 - 1) << 0      // 2 cycle TMRD
                           | (6 - 1) << 4    // 6 cycle TXSR (71.4ns>70ns)
                           | (4 - 1) << 8    // 4 cycle TRAS (47.6ns>42ns)
                           | (2 - 1) << 16   // 2 cycle TWR
                           | (2 - 1) << 24;  // 2 cycle TRCD (23.8ns>15ns)
#else
#error No SDRAM timings for this clock
#endif

    FMC_Bank5_6->SDCMR = FMC_SDCMR_CTB2  // Enable bank 2
                         | 1;            // MODE=001 clock enabled
    sdramCommandWait();

    // ST and SDRAM datasheet agree a 100us delay is required here.
    delayUs(100);

    FMC_Bank5_6->SDCMR = FMC_SDCMR_CTB2  // Enable bank 2
                         | 2;            // MODE=010 precharge all command
    sdramCommandWait();

    FMC_Bank5_6->SDCMR = (8 - 1) << 5      // NRFS=8 SDRAM datasheet says
                                           // "at least two AUTO REFRESH cycles"
                         | FMC_SDCMR_CTB2  // Enable bank 2
                         | 3;              // MODE=011 auto refresh
    sdramCommandWait();

    FMC_Bank5_6->SDCMR = 0x220 << 9  // MRD=0x220:CAS latency=2 burst len=1
                         | FMC_SDCMR_CTB2  // Enable bank 2
                         | 4;              // MODE=100 load mode register
    sdramCommandWait();

// 64ms/4096=15.625us
#ifdef SYSCLK_FREQ_180MHz
    // 15.625us*90MHz=1406-20=1386
    FMC_Bank5_6->SDRTR = 1386 << 1;
#elif defined(SYSCLK_FREQ_168MHz)
    // 15.625us*84MHz=1312-20=1292
    FMC_Bank5_6->SDRTR = 1292 << 1;
#else
#error No refresh timings for this clock
#endif
}

// static IRQDisplayPrint *irq_display;
void IRQbspInit()
{
// If using SDRAM GPIOs are enabled by configureSdram(), else enable them here
#ifndef __ENABLE_XRAM
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN |
                    RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN |
                    RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN;
    RCC_SYNC();
#endif  //__ENABLE_XRAM

    // enable spi1, spi4
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_SPI4EN;

    RCC_SYNC();

    interfaces::spi1::sck::mode(Mode::ALTERNATE);
    interfaces::spi1::sck::alternateFunction(5);
    interfaces::spi1::miso::mode(Mode::ALTERNATE);
    interfaces::spi1::miso::alternateFunction(5);
    interfaces::spi1::mosi::mode(Mode::ALTERNATE);
    interfaces::spi1::mosi::alternateFunction(5);

    interfaces::spi4::sck::mode(Mode::ALTERNATE);
    interfaces::spi4::sck::alternateFunction(5);
    interfaces::spi4::miso::mode(Mode::ALTERNATE);
    interfaces::spi4::miso::alternateFunction(5);
    interfaces::spi4::mosi::mode(Mode::ALTERNATE);
    interfaces::spi4::mosi::alternateFunction(5);

    peripherals::ra01::pc13::cs::mode(Mode::OUTPUT);
    peripherals::ra01::pc13::dio0::mode(Mode::INPUT);
    peripherals::ra01::pc13::nrst::mode(Mode::OUTPUT);
    peripherals::ra01::pc13::cs::high();
    peripherals::ra01::pc13::nrst::high();

    peripherals::ra01::pe4::cs::mode(Mode::OUTPUT);
    peripherals::ra01::pe4::dio0::mode(Mode::INPUT);
    peripherals::ra01::pe4::nrst::mode(Mode::OUTPUT);
    peripherals::ra01::pe4::cs::high();
    peripherals::ra01::pe4::nrst::high();

    peripherals::cc3135::cs::mode(Mode::OUTPUT);
    peripherals::cc3135::hib::mode(Mode::OUTPUT);
    peripherals::cc3135::intr::mode(Mode::INPUT);
    peripherals::cc3135::nrst::mode(Mode::OUTPUT);
    peripherals::cc3135::cs::high();
    peripherals::cc3135::hib::high();
    peripherals::cc3135::nrst::high();

    _led::mode(Mode::OUTPUT);
    ledOn();
    delayMs(100);
    ledOff();
    DefaultConsole::instance().IRQset(intrusive_ref_ptr<Device>(
        new STM32Serial(defaultSerial, defaultSerialSpeed,
                        defaultSerialFlowctrl ? STM32Serial::RTSCTS
                                              : STM32Serial::NOFLOWCTRL)));
    //     irq_display = new IRQDisplayPrint();
    //     DefaultConsole::instance().IRQset(intrusive_ref_ptr<Device>(irq_display));
}

// void* printIRQ(void *argv)
// {
// 	intrusive_ref_ptr<IRQDisplayPrint> irqq(irq_display);
// 	irqq.get()->printIRQ();
// 	return NULL;
// }

void bspInit2()
{
#ifdef WITH_FILESYSTEM
    basicFilesystemSetup(SDIODriver::instance());
#endif  // WITH_FILESYSTEM
    //     Thread::create(printIRQ, 2048);
}

//
// Shutdown and reboot
//

/**
This function disables filesystem (if enabled), serial port (if enabled) and
puts the processor in deep sleep mode.<br>
Wakeup occurs when PA.0 goes high, but instead of sleep(), a new boot happens.
<br>This function does not return.<br>
WARNING: close all files before using this function, since it unmounts the
filesystem.<br>
When in shutdown mode, power consumption of the miosix board is reduced to ~
5uA??, however, true power consumption depends on what is connected to the GPIO
pins. The user is responsible to put the devices connected to the GPIO pin in
the minimal power consumption mode before calling shutdown(). Please note that
to minimize power consumption all unused GPIO must not be left floating.
*/
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
