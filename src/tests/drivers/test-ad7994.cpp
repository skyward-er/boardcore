/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <miosix.h>
#include "drivers/BusTemplate.h"
#include "drivers/adc/AD7994.h"

using namespace miosix;

// #ifdef _BOARD_STM32F429ZI_SKYWARD_HOMEONE

#include <interfaces-impl/hwmapping.h>
using I2CProtocol = ProtocolI2C<miosix::I2C1Driver>;

using convst = miosix::sensors::ad7994::nconvst;
using busy   = miosix::sensors::ad7994::ab;

// // Just for testing using a logic analyzer
// #elif defined _BOARD_STM32F429ZI_STM32F4DISCOVERY

// #include "drivers/SoftwareI2CAdapter.h"

// using scl = Gpio<GPIOB_BASE, 8>;
// using sda = Gpio<GPIOB_BASE, 9>;

// using I2CProtocol = SoftwareI2CAdapter<sda, scl>;

// using busy   = Gpio<GPIOB_BASE, 1>;
// using convst = Gpio<GPIOG_BASE, 9>;

// #endif

typedef AD7994<I2CProtocol, busy, convst> AD7994_t;

int main()
{

    // scl::mode(Mode::OUTPUT);
    // sda::mode(Mode::OUTPUT);

    convst::mode(Mode::OUTPUT);

    // Address for part number AD7994-0, with AS pin HIGH
    AD7994_t ad{0b0100010 << 1};
    if(ad.init())
        printf("Init succeeded\n");
    else
        printf("Init failed\n");

    ad.enableChannel(AD7994_t::Channel::CH1);
    ad.enableChannel(AD7994_t::Channel::CH2);
    ad.enableChannel(AD7994_t::Channel::CH3);
    ad.enableChannel(AD7994_t::Channel::CH4);

    AD7994Sample sample;

    for (;;)
    {
    	if(ad.onSimpleUpdate())
    	{	
    		sample = ad.getLastSample(AD7994_t::Channel::CH1);
        	printf("timestamp: %d value: %d\n", sample.timestamp, sample.value);	
    	}


	    Thread::sleep(1000);
    }
}