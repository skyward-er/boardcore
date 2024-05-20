/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Alberto Nidasio, Davide Mor
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
#include "drivers/serial_stm32.h"
#include "drivers/stm32_sgm.h"
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

static void sdramCommandWait()
{
    for (int i = 0; i < 0xffff; i++)
        if ((FMC_Bank5_6->SDSR & FMC_SDSR_BUSY) == 0)
            return;
}

void configureSdram()
{
    // Enable gpios used by the ram
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN |
                    RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN;
    RCC_SYNC();

    // On the compute unit with F767ZI, the SDRAM pins are:
    // - PG8:  FMC_SDCLK  (sdram clock)
    // - PB5:  FMC_SDCKE1 (sdram bank 2 clock enable)
    // - PB6:  FMC_SDNE1  (sdram bank 2 chip enable)
    // - PF0:  FMC_A0
    // - PF1:  FMC_A1
    // - PF2:  FMC_A2
    // - PF3:  FMC_A3
    // - PF4:  FMC_A4
    // - PF5:  FMC_A5
    // - PF12: FMC_A6
    // - PF13: FMC_A7
    // - PF14: FMC_A8
    // - PF15: FMC_A9
    // - PG0:  FMC_A10
    // - PG1:  FMC_A11
    // - PG2:  FMC_A12    (used only by the 32MB ram, not by the 8MB one)
    // - PD14: FMC_D0
    // - PD15: FMC_D1
    // - PD0:  FMC_D2
    // - PD1:  FMC_D3
    // - PE7:  FMC_D4
    // - PE8:  FMC_D5
    // - PE9:  FMC_D6
    // - PE10: FMC_D7
    // - PE11: FMC_D8
    // - PE12: FMC_D9
    // - PE13: FMC_D10
    // - PE14: FMC_D11
    // - PE15: FMC_D12
    // - PD8:  FMC_D13
    // - PD9:  FMC_D14
    // - PD10: FMC_D15

    // - PG4:  FMC_BA0
    // - PG5:  FMC_BA1
    // - PF11: FMC_SDNRAS
    // - PG15: FMC_SDNCAS
    // - PC0:  FMC_SDNWE
    // - PE0:  FMC_NBL0
    // - PE1:  FMC_NBL1

    // All SDRAM GPIOs needs to be configured with alternate function 12 and
    // maximum speed

    // WARNING: The current configuration is for the 8MB ram

    // Alternate functions
    GPIOB->AFR[0] = 0x0cc00000;
    GPIOC->AFR[0] = 0x0000000c;
    GPIOD->AFR[0] = 0x000000cc;
    GPIOD->AFR[1] = 0xcc000ccc;
    GPIOE->AFR[0] = 0xc00000cc;
    GPIOE->AFR[1] = 0xcccccccc;
    GPIOF->AFR[0] = 0x00cccccc;
    GPIOF->AFR[1] = 0xccccc000;
    GPIOG->AFR[0] = 0x00cc0ccc;
    GPIOG->AFR[1] = 0xc000000c;

    // Mode
    GPIOB->MODER = 0x00002800;
    GPIOC->MODER = 0x00000002;
    GPIOD->MODER = 0xa02a000a;
    GPIOE->MODER = 0xaaaa800a;
    GPIOF->MODER = 0xaa800aaa;
    GPIOG->MODER = 0x80020a2a;

    // Speed (high speed for all, very high speed for SDRAM pins)
    GPIOB->OSPEEDR = 0x00003c00;
    GPIOC->OSPEEDR = 0x00000003;
    GPIOD->OSPEEDR = 0xf03f000f;
    GPIOE->OSPEEDR = 0xffffc00f;
    GPIOF->OSPEEDR = 0xffc00fff;
    GPIOG->OSPEEDR = 0xc0030f3f;

    // Since we'we un-configured PB3 and PB4 (by default they are SWO and NJRST)
    // finish the job and remove the default pull-up
    GPIOB->PUPDR = 0;

    // Enable the SDRAM controller clock
    RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;
    RCC_SYNC();

    // The SDRAM is a AS4C16M16SA-6TIN
    // 16Mx16bit = 256Mb = 32MB
    // HCLK = 216MHz -> SDRAM clock = HCLK/2 = 108MHz

    // 1. Memory device features
    FMC_Bank5_6->SDCR[0] = 0                     // 0 delay after CAS latency
                           | FMC_SDCR1_RBURST    // Enable read bursts
                           | FMC_SDCR1_SDCLK_1;  // SDCLK = HCLK / 2
    FMC_Bank5_6->SDCR[1] = 0                     // Write accesses allowed
                           | FMC_SDCR2_CAS_1     // 2 cycles CAS latency
                           | FMC_SDCR2_NB        // 4 internal banks
                           | FMC_SDCR2_MWID_0    // 16 bit data bus
                           | FMC_SDCR2_NR_1      // 13 bit row address
                           | FMC_SDCR2_NC_0;     // 9 bit column address

// 2. Memory device timings
#ifdef SYSCLK_FREQ_216MHz
    // SDRAM timings. One clock cycle is 9.26ns
    FMC_Bank5_6->SDTR[0] =
        (2 - 1) << FMC_SDTR1_TRP_Pos     // 2 cycles TRP  (18.52ns > 18ns)
        | (7 - 1) << FMC_SDTR1_TRC_Pos;  // 7 cycles TRC  (64.82ns > 60ns)
    FMC_Bank5_6->SDTR[1] =
        (2 - 1) << FMC_SDTR1_TRCD_Pos     // 2 cycles TRCD (18.52ns > 18ns)
        | (2 - 1) << FMC_SDTR1_TWR_Pos    // 2 cycles TWR  (min 2cc > 12ns)
        | (5 - 1) << FMC_SDTR1_TRAS_Pos   // 5 cycles TRAS (46.3ns  > 42ns)
        | (7 - 1) << FMC_SDTR1_TXSR_Pos   // 7 cycles TXSR (74.08ns > 61.5ns)
        | (2 - 1) << FMC_SDTR1_TMRD_Pos;  // 2 cycles TMRD (min 2cc > 12ns)
#else
#error No SDRAM timings for this clock
#endif

    // 3. Enable the bank 2 clock
    FMC_Bank5_6->SDCMR =
        0b001 << FMC_SDCMR_MODE_Pos  // Clock Configuration Enable
        | FMC_SDCMR_CTB2;            // Bank 2
    sdramCommandWait();

    // 4. Wait during command execution
    delayUs(100);

    // 5. Issue a "Precharge All" command
    FMC_Bank5_6->SDCMR = 0b010 << FMC_SDCMR_MODE_Pos  // Precharge all
                         | FMC_SDCMR_CTB2;            // Bank 2
    sdramCommandWait();

    // 6. Issue Auto-Refresh commands
    FMC_Bank5_6->SDCMR = 0b011 << FMC_SDCMR_MODE_Pos       // Auto-Refresh
                         | FMC_SDCMR_CTB2                  // Bank 2
                         | (8 - 1) << FMC_SDCMR_NRFS_Pos;  // 8 Auto-Refresh
    sdramCommandWait();

    // 7. Issue a Load Mode Register command
    FMC_Bank5_6->SDCMR =
        0b100 << FMC_SDCMR_MODE_Pos          // Load mode register
        | FMC_SDCMR_CTB2                     // Bank 2
        | 0 << FMC_SDCMR_MRD_Pos             // Burst length = 1
        | (0b010 << 4) << FMC_SDCMR_MRD_Pos  // CAS = 2 clocks,
        | (1 << 9) << FMC_SDCMR_MRD_Pos;     // Single bit write burst mode
    sdramCommandWait();

// 8. Program the refresh rate (4K / 32ms)
// 64ms / 8192 = 7.8125us
#ifdef SYSCLK_FREQ_216MHz
    // 7.8125us * 133MHz = 1039 - 20 = 1019
    FMC_Bank5_6->SDRTR = 1019 << FMC_SDRTR_COUNT_Pos;
#else
#error No SDRAM refresh timings for this clock
#endif
}

void IRQbspInit()
{
    // Enable USART1 pins port
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    userLed1::mode(Mode::OUTPUT);
    userLed2::mode(Mode::OUTPUT);
    userLed3::mode(Mode::OUTPUT);
    userLed4::mode(Mode::OUTPUT);

    interfaces::spi1::sck::mode(Mode::ALTERNATE);
    interfaces::spi1::sck::alternateFunction(5);
    interfaces::spi1::miso::mode(Mode::ALTERNATE);
    interfaces::spi1::miso::alternateFunction(5);
    interfaces::spi1::mosi::mode(Mode::ALTERNATE);
    interfaces::spi1::mosi::alternateFunction(5);

    interfaces::spi3::sck::mode(Mode::ALTERNATE);
    interfaces::spi3::sck::alternateFunction(6);
    interfaces::spi3::miso::mode(Mode::ALTERNATE);
    interfaces::spi3::miso::alternateFunction(6);
    interfaces::spi3::mosi::mode(Mode::ALTERNATE);
    interfaces::spi3::mosi::alternateFunction(5);

    interfaces::spi4::sck::mode(Mode::ALTERNATE);
    interfaces::spi4::sck::alternateFunction(5);
    interfaces::spi4::miso::mode(Mode::ALTERNATE);
    interfaces::spi4::miso::alternateFunction(5);
    interfaces::spi4::mosi::mode(Mode::ALTERNATE);
    interfaces::spi4::mosi::alternateFunction(5);

    interfaces::spi6::sck::mode(Mode::ALTERNATE);
    interfaces::spi6::sck::alternateFunction(5);
    interfaces::spi6::miso::mode(Mode::ALTERNATE);
    interfaces::spi6::miso::alternateFunction(5);
    interfaces::spi6::mosi::mode(Mode::ALTERNATE);
    interfaces::spi6::mosi::alternateFunction(5);

    // We do not need to setup the miosix usart

    interfaces::uart4::tx::mode(Mode::ALTERNATE);
    interfaces::uart4::tx::alternateFunction(8);
    interfaces::uart4::rx::mode(Mode::ALTERNATE);
    interfaces::uart4::rx::alternateFunction(8);

    interfaces::i2c1::sda::mode(Mode::ALTERNATE);
    interfaces::i2c1::sda::alternateFunction(4);
    interfaces::i2c1::scl::mode(Mode::ALTERNATE);
    interfaces::i2c1::scl::alternateFunction(4);

    interfaces::can1::tx::mode(Mode::ALTERNATE);
    interfaces::can1::tx::alternateFunction(9);
    interfaces::can1::rx::mode(Mode::ALTERNATE);
    interfaces::can1::rx::alternateFunction(9);

    interfaces::timers::tim1ch1::mode(Mode::ALTERNATE);
    interfaces::timers::tim1ch1::alternateFunction(1);
    interfaces::timers::tim3ch1::mode(Mode::ALTERNATE);
    interfaces::timers::tim3ch1::alternateFunction(2);
    interfaces::timers::tim4ch2::mode(Mode::ALTERNATE);
    interfaces::timers::tim4ch2::alternateFunction(2);
    interfaces::timers::tim9ch1::mode(Mode::ALTERNATE);
    interfaces::timers::tim9ch1::alternateFunction(3);

    interfaces::adcs::adc12in8::mode(Mode::INPUT_ANALOG);
    interfaces::adcs::adc12in9::mode(Mode::INPUT_ANALOG);
    interfaces::adcs::adc123in11::mode(Mode::INPUT_ANALOG);

    sensors::ADS131M08::cs::mode(Mode::OUTPUT);
    sensors::ADS131M08::cs::high();
    sensors::LPS22DF::cs::mode(Mode::OUTPUT);
    sensors::LPS22DF::cs::high();
    sensors::LIS2MDL::cs::mode(Mode::OUTPUT);
    sensors::LIS2MDL::cs::high();
    sensors::H3LIS331DL::cs::mode(Mode::OUTPUT);
    sensors::H3LIS331DL::cs::high();
    sensors::LSM6DSRX::cs::mode(Mode::OUTPUT);
    sensors::LSM6DSRX::cs::high();
    sensors::UBXGps::cs::mode(Mode::OUTPUT);
    sensors::UBXGps::cs::high();
    sensors::PS0::cs::mode(Mode::OUTPUT);
    sensors::PS0::cs::high();
    sensors::PS1::cs::mode(Mode::OUTPUT);
    sensors::PS1::cs::high();
    sensors::PS2::cs::mode(Mode::OUTPUT);
    sensors::PS2::cs::high();

    sensors::VN100::cs::mode(Mode::OUTPUT);
    sensors::VN100::cs::high();
    sensors::VN100::syncIn::mode(Mode::INPUT);
    sensors::VN100::syncOut::mode(Mode::OUTPUT);
    sensors::VN100::syncOut::low();

    radio::cs::mode(Mode::OUTPUT);
    radio::cs::high();
    radio::txEn::mode(Mode::OUTPUT);
    radio::txEn::low();
    radio::rxEn::mode(Mode::OUTPUT);
    radio::rxEn::low();
    radio::nrst::mode(Mode::OUTPUT);
    radio::nrst::high();
    radio::dio0::mode(Mode::INPUT_PULL_UP);
    radio::dio1::mode(Mode::INPUT_PULL_UP);
    radio::dio3::mode(Mode::INPUT_PULL_UP);

    sense::detachPayload::mode(Mode::INPUT);
    sense::detachMain::mode(Mode::INPUT);
    sense::detachRamp::mode(Mode::INPUT);
    sense::expulsionSense::mode(Mode::INPUT);

    gpios::mainDeploy::mode(Mode::OUTPUT);
    gpios::mainDeploy::low();
    gpios::boardLed::mode(Mode::OUTPUT);
    gpios::boardLed::low();
    gpios::statusLed::mode(Mode::OUTPUT);
    gpios::statusLed::low();
    gpios::camEnable::mode(Mode::OUTPUT);
    gpios::camEnable::low();

    DefaultConsole::instance().IRQset(intrusive_ref_ptr<Device>(new STM32Serial(
        defaultSerial, defaultSerialSpeed, STM32Serial::NOFLOWCTRL)));
}

void bspInit2()
{
#ifdef WITH_FILESYSTEM
    basicFilesystemSetup(SDIODriver::instance());
#endif  // WITH_FILESYSTEM

#ifdef WITH_BACKUP_SRAM
    // Print the reset reason
    bootlog("Reset reson: ");
    switch (SGM::instance().lastResetReason())
    {
        case ResetReason::RST_LOW_PWR:
            bootlog("low power\n");
            break;
        case ResetReason::RST_WINDOW_WDG:
            bootlog("window watchdog\n");
            break;
        case ResetReason::RST_INDEPENDENT_WDG:
            bootlog("indeendent watchdog\n");
            break;
        case ResetReason::RST_SW:
            bootlog("software reset\n");
            break;
        case ResetReason::RST_POWER_ON:
            bootlog("power on\n");
            break;
        case ResetReason::RST_PIN:
            bootlog("reset pin\n");
            break;
        case ResetReason::RST_UNKNOWN:
            bootlog("unknown\n");
            break;
    }
#endif  // WITH_BACKUP_SRAM
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
