/* Copyright (c) 2023 Skyward Experimental Rocketry
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

/**
 * This is a quick and dirty test of the QSPI flash on board the compute unit. A
 * proper driver for the flash will need to be developed!
 */

#include <miosix.h>
#include <utils/ClockUtils.h>

using namespace miosix;

/**
 * QSPI Flash pins
 *
 * FLASH_NSS - PB10 - AF9  - QUADSPI_BK1_NCS
 * FLASH_CLK - PF10 - AF9  - QUADSPI_CLK
 * FLASH_IO0 - PF8  - AF10 - QUADSPI_BK1_IO0
 * FLASH_IO1 - PF9  - AF10 - QUADSPI_BK1_IO1
 * FLASH_IO2 - PF7  - AF9  - QUADSPI_BK1_IO2
 * FLASH_IO3 - PF6  - AF9  - QUADSPI_BK1_IO3
 */

GpioPin flash_ncs(GPIOB_BASE, 10);
GpioPin flash_sck(GPIOF_BASE, 10);
GpioPin flash_io0(GPIOF_BASE, 8);
GpioPin flash_io1(GPIOF_BASE, 9);
GpioPin flash_io2(GPIOF_BASE, 7);
GpioPin flash_io3(GPIOF_BASE, 6);

#include <miosix.h>

using namespace miosix;

int main()
{
    flash_ncs.mode(Mode::ALTERNATE);
    flash_ncs.alternateFunction(9);
    flash_ncs.speed(Speed::_100MHz);
    flash_sck.mode(Mode::ALTERNATE);
    flash_sck.alternateFunction(9);
    flash_sck.speed(Speed::_100MHz);
    flash_io0.mode(Mode::ALTERNATE);
    flash_io0.alternateFunction(10);
    flash_io0.speed(Speed::_100MHz);
    flash_io1.mode(Mode::ALTERNATE);
    flash_io1.alternateFunction(10);
    flash_io1.speed(Speed::_100MHz);
    flash_io2.mode(Mode::ALTERNATE);
    flash_io2.alternateFunction(9);
    flash_io2.speed(Speed::_100MHz);
    flash_io3.mode(Mode::ALTERNATE);
    flash_io3.alternateFunction(9);
    flash_io3.speed(Speed::_100MHz);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOFEN;
    RCC->AHB3ENR |= RCC_AHB3ENR_QSPIEN;

    RCC_SYNC();

    delayMs(2 * 1000);
    printf("Starting\n");

    QUADSPI->CR |= QUADSPI_CR_ABORT;  // Abort ongoing commands

    // Wait while aborted
    while (QUADSPI->CR & QUADSPI_CR_ABORT)
        ;

    // Reset
    QUADSPI->CR  = 0;
    QUADSPI->DCR = 0;
    QUADSPI->CCR = 0;

    // QSPI peripheral initialization
    QUADSPI->CR |=
        QUADSPI_CR_SSHIFT |             // Wait a full cycle to read
        3 << QUADSPI_CR_PRESCALER_Pos;  // QSPI clock = 216MHz / 3 = 72MHz
    // QUADSPI->DCR |=
    //     21 << QUADSPI_DCR_FSIZE_Pos;  // Flash size 32Mb = 4MB = 2^(21+1)
    //     bytes

    // Enable the peripheral
    QUADSPI->CR |= QUADSPI_CR_EN;

    // Send read ID command - 0x9Fcl
    {
        QUADSPI->CCR |= 1 << QUADSPI_CCR_FMODE_Pos |   // Indirect read mode
                        1 << QUADSPI_CCR_DMODE_Pos |   // Data on 1-wire
                        0 << QUADSPI_CCR_ABMODE_Pos |  // No alternate bytes
                        0 << QUADSPI_CCR_ADMODE_Pos |  // No address
                        1 << QUADSPI_CCR_IMODE_Pos;    // Instruction on 1-wire

        QUADSPI->DLR = 23;  // Expect to receive 24 bytes

        printf("CCR: %lx\n", QUADSPI->CCR);

        // Trigger communication start by writing the instruction
        QUADSPI->CCR |= 0x9F << QUADSPI_CCR_INSTRUCTION_Pos;

        // Wait for the transaction to complete, and disable the peripheral.
        int count = 0;
        while (QUADSPI->SR & QUADSPI_SR_BUSY)
            count++;

        // Disable the peripheral
        QUADSPI->CR &= ~QUADSPI_CR_EN;

        printf("Data: 0x%lx %d\n", QUADSPI->DR, count);
    }

    while (true)
        Thread::sleep(1000);
}
