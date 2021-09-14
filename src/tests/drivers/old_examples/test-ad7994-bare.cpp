/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <miosix.h>
#include <cstdint>
#include <cstdio>
#include "drivers/BusTemplate.h"

using namespace miosix;

const uint8_t ADDRESS    = 0x22 << 1;
const uint8_t REG_CONFIG = 0x02;
const uint8_t DEFAULT_CONFIG = 0x08;

typedef ProtocolI2C<I2C1Driver> my_i2c1;

int main()
{
    // soft_i2c::init();
    my_i2c1::init();

    uint8_t reg_config = REG_CONFIG;
    uint8_t config_val = 0x0A;
    uint8_t config_read_val;
    for (;;)
    {
        // Read default value
        my_i2c1::directWrite(ADDRESS, &reg_config, 1);
        my_i2c1::directRead(ADDRESS, &config_read_val, 1);

        if (config_read_val == DEFAULT_CONFIG)
        {
            printf("1. Read ok.\n");
        }
        else
        {
            printf("1. ERROR. read: %d\n", config_read_val);
        }
        Thread::sleep(200);
        // Write custom value and read back.
        my_i2c1::write(ADDRESS, REG_CONFIG, &config_val, 1);
        my_i2c1::directRead(ADDRESS, &config_read_val, 1);

        if (config_read_val == config_val)
        {
            printf("2. Read ok.\n");
        }
        else
        {
            printf("2. ERROR. read: %d\n", config_read_val);
        }

        Thread::sleep(200);
        // Read back the wrong way
        config_read_val = 0;
        my_i2c1::read(ADDRESS, REG_CONFIG, &config_read_val, 1);

        if (config_read_val != config_val)
        {
            printf("3. OK. Reading the wrong way doesn't work as expected\n");
        }
        else
        {
            printf("3. ERROR. Succesfully read when it should't\n");
        }
        printf("\n\n");
        Thread::sleep(1500);
    }
}