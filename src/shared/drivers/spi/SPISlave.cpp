/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include "SPISlave.h"

#include <miosix.h>

namespace Boardcore
{

SPISlave::SlaveSelect SPISlave::SlaveSelect::MuxSelect(GpioType s0, GpioType s1,
                                                       GpioType s2,
                                                       uint8_t value)
{
    return SlaveSelect{
        .type = Type::MUX,
        .data =
            {
                .mux =
                    {
                        .s0    = s0,
                        .s1    = s1,
                        .s2    = s2,
                        .value = value,
                    },
            },
    };
}

SPISlave::SlaveSelect SPISlave::SlaveSelect::PinSelect(GpioType cs)
{
    return SlaveSelect{
        .type = Type::CS,
        .data =
            {
                .cs = cs,
            },
    };
}

void SPISlave::SlaveSelect::select()
{
    switch (type)
    {
        case Type::CS:
            data.cs.low();
            break;

        case Type::MUX:
            if (data.mux.value & 0b001)
                data.mux.s0.high();
            else
                data.mux.s0.low();

            if (data.mux.value & 0b010)
                data.mux.s1.high();
            else
                data.mux.s1.low();

            if (data.mux.value & 0b100)
                data.mux.s2.high();
            else
                data.mux.s2.low();

            break;
    }
}

void SPISlave::SlaveSelect::deselect()
{
    switch (type)
    {
        case Type::CS:
            data.cs.high();
            break;

        case Type::MUX:
            data.mux.s0.low();
            data.mux.s1.low();
            data.mux.s2.low();

            break;
    }
}

void SPISlave::select()
{
    cs.select();

    if (config.csSetupTimeUs > 0)
    {
        miosix::delayUs(config.csSetupTimeUs);
    }
}

void SPISlave::deselect()
{
    if (config.csHoldTimeUs > 0)
    {
        miosix::delayUs(config.csHoldTimeUs);
    }

    cs.deselect();
}
}  // namespace Boardcore