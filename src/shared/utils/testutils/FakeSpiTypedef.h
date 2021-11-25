/* Copyright (c) 2020 Skyward Experimental Rocketry
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

#pragma once
#include <miosix.h>

#include <cstdint>
#include <vector>

#include "utils/testutils/MockGpioPin.h"

using std::vector;

/**
 * @brief Mock STM32F4 SPI peripheral: intercepts register value changes to
 * emulate a real SPI peripheral / slave.
 */
struct FakeSpiTypedef
{
    uint32_t CR1    = 0;
    uint32_t CR2    = 0;
    uint32_t SR     = 3;
    uint32_t RXCRCR = 0;
    uint32_t TXCRCR = 0;

    struct RegDR
    {
        // Intercept uint32_t assignements
        void operator=(uint32_t DR)
        {
            // If slave is selected & bus configured properly
            if (parent.cs.value() == 0 && parent.CR1 == parent.CR1_expected &&
                parent.CR2 == parent.CR2_expected)
            {
                out_buf.push_back(DR);
            }
        }

        operator uint32_t()
        {
            // If slave is selected
            if (parent.cs.value() == 0 && parent.CR1 == parent.CR1_expected &&
                parent.CR2 == parent.CR2_expected)
            {
                return in_buf[in_it++];
            }

            return 0;
        }

        RegDR(FakeSpiTypedef& parent) : parent(parent) {}

        unsigned int in_it = 0;
        vector<uint32_t> in_buf;
        vector<uint32_t> out_buf;

    private:
        FakeSpiTypedef& parent;
    };

    uint32_t CR1_expected = 0;
    uint32_t CR2_expected = 0;

    RegDR DR;
    MockGpioPin cs;

    FakeSpiTypedef() : DR(*this) { cs.high(); }
};