/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Nuno Barcellos
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
#include "drivers/BusTemplate.h"
#include "drivers/adc/AD7994.h"

using namespace miosix;

// Homeone and deathstack already define pins
#include <interfaces-impl/hwmapping.h>
using I2CProtocol = ProtocolI2C<miosix::I2C1Driver>;

using convst = miosix::sensors::ad7994::nconvst;
using busy   = miosix::sensors::ad7994::ab;
static constexpr uint8_t addr = miosix::sensors::ad7994::addr;


typedef AD7994<I2CProtocol, busy, convst> AD7994_t;

int main()
{
    convst::mode(Mode::OUTPUT);

    AD7994_t ad{addr};
    if(ad.init())
        printf("Init succeeded\n");
    else
        printf("Init failed\n");

    ad.enableChannel(AD7994_t::Channel::CH1);
    ad.enableChannel(AD7994_t::Channel::CH2);
    ad.enableChannel(AD7994_t::Channel::CH3);
    ad.enableChannel(AD7994_t::Channel::CH4);

    AD7994Sample sample1, sample2, sample3, sample4;

    for (;;)
    {
        if(ad.onSimpleUpdate())
        {
            sample1 = ad.getLastSample(AD7994_t::Channel::CH1);
            sample2 = ad.getLastSample(AD7994_t::Channel::CH2);
            sample3 = ad.getLastSample(AD7994_t::Channel::CH3);
            sample4 = ad.getLastSample(AD7994_t::Channel::CH4);
            printf("timestamp: %d value: %d\n", (int)sample1.timestamp, (int)sample1.value);
            printf("timestamp: %d value: %d\n", (int)sample2.timestamp, (int)sample2.value);
            printf("timestamp: %d value: %d\n", (int)sample3.timestamp, (int)sample3.value);
            printf("timestamp: %d value: %d\n\n", (int)sample4.timestamp, (int)sample4.value);
        }

        Thread::sleep(1000);
    }
}