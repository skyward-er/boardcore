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
#include <util/software_i2c.h>
#include "drivers/BusTemplate.h"

using namespace miosix;

const uint8_t ADDRESS      = 0x68 << 1;
const uint8_t WHO_AM_I     = 0x73;
const uint8_t REG_WHO_AM_I = 117;  // Register 117

typedef ProtocolI2C<I2C1Driver> my_i2c1;

typedef Gpio<GPIOB_BASE, 8> scl;
typedef Gpio<GPIOB_BASE, 9> sda;

using soft_i2c = SoftwareI2C<sda, scl>;


void masterRead(uint8_t bit7_address, uint8_t reg_address,
                         uint8_t *data, int len)
{
    int i = 0;
    soft_i2c::sendStart();
    soft_i2c::send((bit7_address));
    soft_i2c::send(reg_address);
    soft_i2c::sendRepeatedStart();
    soft_i2c::send((bit7_address) + 1);

    for (i      = 0; i < (len - 1); i++)
        data[i] = soft_i2c::recvWithAck();

    data[len - 1] = soft_i2c::recvWithNack();

    soft_i2c::sendStop();
}

int main()
{
   // soft_i2c::init();
   
    my_i2c1::init();

    uint8_t who_am_i;
    for (;;)
    {
        long long s = miosix::getTick();
        my_i2c1::read(ADDRESS, REG_WHO_AM_I, &who_am_i, 1);
        long long d = miosix::getTick() - s;
        
        if (who_am_i == WHO_AM_I)
        {
            printf("Read ok. (%d)\n", (int)d);
        }
        else
        {
            printf("ERROR. read: %d\n", who_am_i);
        }
        Thread::sleep(1500);
    }
}