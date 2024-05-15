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

qspi_flash mymemory;

int main()
{

    // change API from std::vector to normal arrays with no dynamic allocation.
    // sector_erase() timeout values increased due to some problems
    //  with write_vector().
    // read_sector and write_vector tested !

    // init qspi-flash communication
    mymemory.init();

    // test if flash is working
    if (mymemory.test())
    {

        // read device id
        printf("\nID: %x\n", mymemory.readID());

        // create a vector
        const size_t vect_size  = 5000;
        uint8_t vect[vect_size] = {0};
        uint32_t i              = 0;
        for (i = 0; i < vect_size; i++)
        {
            vect[i] = 77;
        }

        // write vector "vect"
        if (mymemory.write_vector(vect, vect_size, 883, true) == false)
        {
            printf("ERROR - write operation failed !\n");
            return -1;
        }

        printf("write operaton succeded!\n");

        uint8_t b[6000] = {0};
        printf("read_sector: %d\n", mymemory.read_sector(b, 6000, 884));

        printf("array (b): \n");
        for (i = 0; i < 6000; i++)
        {
            printf("b[%d]: %d\n", i, b[i]);
        }

        return 0;
    }
}