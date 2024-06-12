/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Valerio Flamminii
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
#include "drivers/qspi-flash/qspi-flash.h"

/*
 * This is a simple test of "MX25R3235FM1IL0" flash memory chip on compute unit.
 * it will ensure that flash is working by performing "write" and "read" vector
 * operations.
 */

QspiFlash mymemory(QUADSPI);

// init all GPIO pins needed to communicate with flash memory
void initBoard();

int main()
{

    initBoard();

    // init qspi-flash communication
    mymemory.init();

    // test if flash is working
    if (mymemory.test())
    {

        // read device id
        printf("\nID: %ld\n", mymemory.readID());

        // erase block
        printf("\nerase block64: %d\n",
               mymemory.block64Erase(FlashMemory::BLOCK64_SIZE * 60));

        // create a vector
        const size_t vect_size  = 5000;
        uint8_t vect[vect_size] = {0};
        uint32_t i              = 0;
        for (i = 0; i < vect_size; i++)
        {
            vect[i] = i % 12;
        }

        // write vector "vect"
        if (mymemory.write(vect, vect_size, 1022, true) == false)
        {
            printf("ERROR - write operation failed !\n");
            return -1;
        }

        printf("write operaton succeded!\n");

        uint8_t b[5000] = {0};
        uint32_t a      = 0;
        for (a = 0; a < 5000; a++)
        {
            b[a] = 55;
        }
        printf("read_sector: %d\n", mymemory.readSector(b, 5000, 1022));

        printf("array (b): \n");
        for (i = 0; i < 5000; i++)
        {
            printf("b[%ld]: %d\n", i, b[i]);
        }

        return 0;
    }
}

void initBoard()
{

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

    // init GPIO peripheral pins
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
}