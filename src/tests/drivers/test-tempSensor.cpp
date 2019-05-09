/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: 
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <Common.h>
#include "sensors/LM75B.h"
#include "drivers/BusTemplate.h"

using namespace miosix;

#include <interfaces-impl/hwmapping.h>
using I2CProtocol = ProtocolI2C<miosix::I2C1Driver>;

uint8_t addr1 = 0x48 << 1;
uint8_t addr2 = 0x49 << 1;

typedef LM75B<I2CProtocol> LM75BType;
int main()
{
    LM75BType temp1{addr1};
    LM75BType temp2{addr2};
    Thread::sleep(500);

    while (true)
    {
        bool result1 = temp1.selfTest();
        bool result2 = temp2.selfTest();

        miosix::ledOn();
        TRACE("LM75B self test result: temp1=%d temp2=%d\n", result1, result2);
        Thread::sleep(500);
        miosix::ledOff();
        Thread::sleep(500);
        TRACE("LM75B (1) temperature: %f\n", temp1.getTemp());
        TRACE("LM75B (2) temperature: %f\n", temp2.getTemp());
    }
}
