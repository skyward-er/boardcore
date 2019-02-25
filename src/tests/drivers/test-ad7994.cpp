/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#ifdef _BOARD_STM32F429ZI_SKYWARD_HOMEONE

#include <interfaces-impl/hwmapping.h>
using I2CProtocol = ProtocolI2C<miosix::I2C1Driver>;

using convst = miosix::sensors::ad7994::nconvst;
using busy   = miosix::sensors::ad7994::ab;

// Just for testing using a logic analyzer
#elif defined _BOARD_STM32F429ZI_STM32F4DISCOVERY

#include "drivers/SoftwareI2CAdapter.h"

using scl = Gpio<GPIOA_BASE, 8>;
using sda = Gpio<GPIOC_BASE, 9>;

using I2CProtocol = ProtocolI2C<SoftwareI2CAdapter<sda, scalbln> >;

using busy   = Gpio<GPIOC_BASE, 8>;
using convst = Gpio<GPIOB_BASE, 7>;

#endif

int main()
{
    convst::mode(Mode::OUTPUT);

    AD7994<I2CProtocol, busy, convst> ad{123};
    for (;;)
    {
        ad.init();
    }
}