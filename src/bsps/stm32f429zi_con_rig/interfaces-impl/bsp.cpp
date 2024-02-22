/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI6EN;

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;

    RCC_SYNC();

    // Initialize everything
    interfaces::spi1::sck::mode(Mode::ALTERNATE);
    interfaces::spi1::sck::alternateFunction(5);
    interfaces::spi1::miso::mode(Mode::ALTERNATE);
    interfaces::spi1::miso::alternateFunction(5);
    interfaces::spi1::mosi::mode(Mode::ALTERNATE);
    interfaces::spi1::mosi::alternateFunction(5);

    btns::ignition::mode(Mode::INPUT);
    btns::filling::mode(Mode::INPUT);
    btns::venting::mode(Mode::INPUT);
    btns::release::mode(Mode::INPUT);
    btns::detach::mode(Mode::INPUT);
    btns::tars::mode(Mode::INPUT);
    btns::arm::mode(Mode::INPUT);

    radio::cs::mode(Mode::OUTPUT);
    radio::cs::high();
    radio::dio0::mode(Mode::INPUT_PULL_UP);
    radio::dio1::mode(Mode::INPUT_PULL_UP);
    radio::dio3::mode(Mode::INPUT_PULL_UP);
    radio::txEn::mode(Mode::OUTPUT);
    radio::txEn::low();
    radio::rxEn::mode(Mode::OUTPUT);
    radio::rxEn::low();
    radio::nrst::mode(Mode::OUTPUT);
    radio::nrst::high();

    ui::buzzer::mode(Mode::OUTPUT);
    ui::buzzer::high();
    ui::armedLed::mode(Mode::OUTPUT);
    ui::armedLed::low();
    ui::redLed::mode(Mode::OUTPUT);
    ui::redLed::low();

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
 * @brief This function disables filesystem and serial port.
 */
void shutdown()
{
    ioctl(STDOUT_FILENO, IOCTL_SYNC, 0);

#ifdef WITH_FILESYSTEM
    FilesystemManager::instance().umountAll();
#endif  // WITH_FILESYSTEM

    disableInterrupts();

    while (true)
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
