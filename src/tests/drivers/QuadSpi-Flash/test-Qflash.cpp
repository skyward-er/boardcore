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
    // aggiunti software_reset con timeout: disaccoppiano le operazioni, anche
    // senza delay - permettendo alle operazioni di erase che falliscono di non
    // far fallire le altre operazioni successive. chip_erase se fallisce dopo
    // circa 10 sec fa fallire alcune operazioni come write_vector() e
    // page_program.

    // init qspi-flash communication
    mymemory.init();

    // test if flash is working
    if (mymemory.test())
    {

        // read device id
        printf("\nID: %x\n", mymemory.readID());

        // create a vector of ten bytes to write on memory
        std::vector<uint8_t> v;
        v.reserve(10);
        for (uint32_t i = 0; i < v.capacity(); i++)
            v.push_back(99);
        v.resize(v.capacity());  // make sure that size match with capacity

        // write vector "v" at 50th sector and double check on data
        if (mymemory.write_vector(v, 88, true) == false)
        {
            printf("ERROR - write operation failed !\n");
            return -1;
        }

        // create a vector in which will be copied the bytes of 50th sector
        std::vector<uint8_t> v2;

        // read 50th entire sector into the vector "v2"
        if (mymemory.read_sector(v2, 88) == false)
        {
            printf("ERROR - read operation failed ! \n");
            return -1;
        }

        // print first ten bytes of data in the vector "v2"
        printf("\nvector v2: \n");
        uint32_t a = 0;
        for (a = 0; a < 10 && a < v2.size(); a++)
        {
            printf("v2[%d]: %d \n", a, v2[a]);
            if (v2[a] != 99)
            {
                printf("ERROR - data mismatch !\n");
                return -1;
            }
        }
        printf("v2 size: %d\n", v2.size());
        printf("v2 capacity: %d\n", v2.capacity());
        printf("\n- flash memory is working properly !\n");
        return 0;
    }
    else
    {
        printf("Error - flash memory is not working properly!\n");
        return -1;
    }
}