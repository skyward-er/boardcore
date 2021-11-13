/* Copyright (c) 2021 Skyward Experimental Rocketry
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <miosix.h>
#include <cstdint>

class ClockFrequency
{
public:
    static uint32_t APB1()
    {
        // The global variable SystemCoreClock from ARM's CMSIS allows to
        // know
        // the CPU frequency.
        uint32_t freq = SystemCoreClock;

// The position of the PPRE1 bit in RCC->CFGR is different in some stm32
#ifdef _ARCH_CORTEXM3_STM32
        const uint32_t ppre1 = 8;
#else  // stm32f2 and f4
        const uint32_t ppre1 = 10;
#endif
        // The timer frequency may however be a submultiple of the CPU
        // frequency, due to the bus at whch the periheral is connected being
        // slower. The RCC->CFGR register tells us how slower the APB1 bus is
        // running.

        if (RCC->CFGR & RCC_CFGR_PPRE1_2)
            freq /= 1 << (((RCC->CFGR >> ppre1) & 0x3) + 1);

        return freq;
    }

    static uint32_t APB2()
    {
        // The global variable SystemCoreClock from ARM's CMSIS allows to
        // know
        // the CPU frequency.
        uint32_t freq = SystemCoreClock;

// The position of the PPRE1 bit in RCC->CFGR is different in some stm32
#ifdef _ARCH_CORTEXM3_STM32
        const uint32_t ppre2 = 11;
#else  // stm32f2 and f4
        const uint32_t ppre2 = 13;
#endif
        // The peripheral frequency may however be a submultiple of the CPU
        // frequency, due to the bus at whch the periheral is connected being
        // slower. The RCC->CFGR register tells us how slower the APB1 or APB2
        // bus are running.

        if (RCC->CFGR & RCC_CFGR_PPRE2_2)
            freq /= 1 << (((RCC->CFGR >> ppre2) & 0x3) + 1);

        return freq;
    }
};