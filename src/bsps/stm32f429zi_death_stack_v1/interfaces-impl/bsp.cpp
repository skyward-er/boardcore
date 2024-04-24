/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Silvano Seva, Alberto Nidasio
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

#ifdef SDRAM_ISSI

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

#else

void configureSdram()
{
    // Enable all gpios
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

    // SDRAM is a AS4C4M16SA-6TAN, connected to bank 2 (0xd0000000)
    // Some bits in SDCR[1] are don't care, and the have to be set in SDCR[0],
    // they aren't just don't care, the controller will fail if they aren't at 0
    FMC_Bank5_6->SDCR[0] = FMC_SDCR1_SDCLK_1  // SDRAM runs @ half CPU frequency
                           | FMC_SDCR1_RBURST  // Enable read burst
                           | 0;  //  0 delay between reads after CAS
    FMC_Bank5_6->SDCR[1] =
        0                   //  8 bit column address
        | FMC_SDCR1_NR_0    // 12 bit row address
        | FMC_SDCR1_MWID_0  // 16 bit data bus
        | FMC_SDCR1_NB      //  4 banks
        | FMC_SDCR1_CAS_1;  //  2 cycle CAS latency (TCK>9+0.5ns [1])

#ifdef SYSCLK_FREQ_180MHz
    // One SDRAM clock cycle is 11.1ns
    // Some bits in SDTR[1] are don't care, and the have to be set in SDTR[0],
    // they aren't just don't care, the controller will fail if they aren't at 0
    FMC_Bank5_6->SDTR[0] = (6 - 1) << 12     // 6 cycle TRC  (66.6ns>60ns)
                           | (2 - 1) << 20;  // 2 cycle TRP  (22.2ns>18ns)
    FMC_Bank5_6->SDTR[1] = (2 - 1) << 0      // 2 cycle TMRD
                           |
                           (6 - 1) << 4  // 6 cycle TXSR (66.6ns>61.5+0.5ns [1])
                           | (4 - 1) << 8    // 4 cycle TRAS (44.4ns>42ns)
                           | (2 - 1) << 16   // 2 cycle TWR
                           | (2 - 1) << 24;  // 2 cycle TRCD (22.2ns>18ns)
#elif defined(SYSCLK_FREQ_168MHz)
    // One SDRAM clock cycle is 11.9ns
    // Some bits in SDTR[1] are don't care, and the have to be set in SDTR[0],
    // they aren't just don't care, the controller will fail if they aren't at 0
    FMC_Bank5_6->SDTR[0] = (6 - 1) << 12     // 6 cycle TRC  (71.4ns>60ns)
                           | (2 - 1) << 20;  // 2 cycle TRP  (23.8ns>18ns)
    FMC_Bank5_6->SDTR[1] = (2 - 1) << 0      // 2 cycle TMRD
                           |
                           (6 - 1) << 4  // 6 cycle TXSR (71.4ns>61.5+0.5ns [1])
                           | (4 - 1) << 8    // 4 cycle TRAS (47.6ns>42ns)
                           | (2 - 1) << 16   // 2 cycle TWR
                           | (2 - 1) << 24;  // 2 cycle TRCD (23.8ns>18ns)
#else
#error No SDRAM timings for this clock
#endif
    // NOTE [1]: the timings for TCK and TIS depend on rise and fall times
    //(see note 9 and 10 on datasheet). Timings are adjusted accordingly to
    // the measured 2ns rise and fall time

    FMC_Bank5_6->SDCMR = FMC_SDCMR_CTB2  // Enable bank 2
                         | 1;            // MODE=001 clock enabled
    sdramCommandWait();

    // SDRAM datasheet requires 200us delay here (note 11), here we use 10% more
    delayUs(220);

    FMC_Bank5_6->SDCMR = FMC_SDCMR_CTB2  // Enable bank 2
                         | 2;            // MODE=010 precharge all command
    sdramCommandWait();

    // FIXME: note 11 on SDRAM datasheet says extended mode register must be
    // set, but the ST datasheet does not seem to explain how
    FMC_Bank5_6->SDCMR = 0x220 << 9  // MRD=0x220:CAS latency=2 burst len=1
                         | FMC_SDCMR_CTB2  // Enable bank 2
                         | 4;              // MODE=100 load mode register
    sdramCommandWait();

    FMC_Bank5_6->SDCMR = (4 - 1) << 5  // NRFS=8 SDRAM datasheet requires
                                       // a minimum of 2 cycles, here we use 4
                         | FMC_SDCMR_CTB2  // Enable bank 2
                         | 3;              // MODE=011 auto refresh
    sdramCommandWait();

// 32ms/4096=7.8125us, but datasheet says to round that to 7.8us
#ifdef SYSCLK_FREQ_180MHz
    // 7.8us*90MHz=702-20=682
    FMC_Bank5_6->SDRTR = 682 << 1;
#elif defined(SYSCLK_FREQ_168MHz)
    // 7.8us*84MHz=655-20=635
    FMC_Bank5_6->SDRTR = 635 << 1;
#else
#error No refresh timings for this clock
#endif
}

#endif

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

    using namespace interfaces;
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

    // Software I2C
    i2c::scl::high();
    i2c::scl::mode(Mode::OPEN_DRAIN);
    i2c::sda::high();
    i2c::sda::mode(Mode::OPEN_DRAIN);

    uart4::rx::mode(Mode::ALTERNATE);
    uart4::rx::alternateFunction(8);
    uart4::tx::mode(Mode::ALTERNATE);
    uart4::tx::alternateFunction(8);

    can::rx::mode(Mode::ALTERNATE);
    can::rx::alternateFunction(9);
    can::tx::mode(Mode::ALTERNATE);
    can::tx::alternateFunction(9);

    using namespace sensors;
    adis16405::cs::mode(Mode::OUTPUT);
    adis16405::cs::high();
    adis16405::ckIn::mode(Mode::ALTERNATE);
    adis16405::ckIn::alternateFunction(2);
    adis16405::dio1::mode(Mode::INPUT);
    adis16405::rst::mode(Mode::OUTPUT);
    adis16405::rst::high();

    ad7994::ab::mode(Mode::INPUT);
    ad7994::nconvst::mode(Mode::OUTPUT);

    mpu9250::cs::mode(Mode::OUTPUT);
    mpu9250::cs::high();
    mpu9250::intr::mode(Mode::INPUT);

    ms5803::cs::mode(Mode::OUTPUT);
    ms5803::cs::high();

    lsm6ds3h::cs::mode(Mode::OUTPUT);
    lsm6ds3h::cs::high();
    lsm6ds3h::int1::mode(Mode::INPUT);
    lsm6ds3h::int2::mode(Mode::INPUT);

    using namespace inputs;
    vbat::mode(Mode::INPUT_ANALOG);
    lp_dtch::mode(Mode::INPUT);
    btn_open::mode(Mode::INPUT);
    btn_close::mode(Mode::INPUT);

    using namespace nosecone;
    motEn::mode(Mode::OUTPUT);
    motEn::low();

    motP1::mode(Mode::OUTPUT);
    motP1::low();
    motP2::mode(Mode::OUTPUT);
    motP2::low();

    rogP1::mode(Mode::ALTERNATE);
    rogP1::alternateFunction(2);

    rogP2::mode(Mode::ALTERNATE);
    rogP2::alternateFunction(2);

    nc_dtch::mode(Mode::INPUT);
    motor_act::mode(Mode::INPUT);

    motor_3v::mode(Mode::OUTPUT);
    motor_3v::high();

    motor_gnd::mode(Mode::OUTPUT);
    motor_gnd::low();

    using namespace actuators;
    tcPwm::mode(Mode::ALTERNATE);
    tcPwm::alternateFunction(3);

    thCut1::ena::mode(Mode::OUTPUT);
    thCut1::ena::low();
    thCut1::csens::mode(Mode::INPUT_ANALOG);

    thCut2::ena::mode(Mode::OUTPUT);
    thCut2::ena::low();
    thCut2::csens::mode(Mode::INPUT_ANALOG);

    misc::buzz::mode(Mode::OUTPUT);
    misc::buzz::low();

    xbee::cs::mode(Mode::OUTPUT);
    xbee::cs::high();
    xbee::attn::mode(Mode::INPUT_PULL_UP);
    xbee::sleep_status::mode(Mode::INPUT);

    xbee::reset::mode(Mode::OPEN_DRAIN);
    xbee::reset::high();

    // Led blink
    led1::mode(Mode::OUTPUT);

    ledOn();
    delayMs(100);
    ledOff();

    DefaultConsole::instance().IRQset(intrusive_ref_ptr<Device>(
        new STM32Serial(defaultSerial, defaultSerialSpeed,
                        defaultSerialFlowctrl ? STM32Serial::RTSCTS
                                              : STM32Serial::NOFLOWCTRL)));
}

void bspInit2()
{
#ifdef WITH_FILESYSTEM
    // PA2,PA3
    intrusive_ref_ptr<DevFs> devFs =
        basicFilesystemSetup(SDIODriver::instance());
    devFs->addDevice("gps",
                     intrusive_ref_ptr<Device>(new STM32Serial(2, 115200)));
#endif  // WITH_FILESYSTEM
}

//
// Shutdown and reboot
//

/**
 * For safety reasons, we never want the board to shutdown.
 * When requested to shutdown, we reboot instead.
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
