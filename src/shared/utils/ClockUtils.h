/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio, Luca Erbetta
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

#include <interfaces/arch_registers.h>

namespace Boardcore
{

namespace ClockUtils
{

/**
 * @brief Timer input clock.
 */
enum class APB
{
    APB1,
    APB2
};

/**
 * @brief Computes the output frequency for the given APB bus.
 *
 * @param bus Advanced Peripheral Bus
 * @return Prescaler input frequency.
 */
uint32_t getAPBFrequecy(APB bus);

}  // namespace ClockUtils

inline uint32_t ClockUtils::getAPBFrequecy(APB bus)
{
    // The global variable SystemCoreClock from ARM's CMIS allows to know the
    // CPU frequency.
    uint32_t inputFrequency = SystemCoreClock;

    // The timer frequency may be a submultiple of the CPU frequency, due to the
    // bus at which the peripheral is connected being slower.
    // The RCC-ZCFGR register tells us how slower the APB bus is running.
    // The following formula takes into account that if the APB1 clock is
    // divided by a factor of two or grater, the timer is clocked at twice the
    // bus interface.
    if (bus == APB::APB1)
    {
        // The position of the PPRE1 bit in RCC->CFGR is different in some stm32
#ifdef _ARCH_CORTEXM3_STM32
        const uint32_t ppre1 = 8;
#elif _ARCH_CORTEXM4_STM32F4
        const uint32_t ppre1 = 10;
#else
#error "Architecture not supported by TimerUtils"
#endif

        if (RCC->CFGR & RCC_CFGR_PPRE1_2)
        {
            inputFrequency /= 1 << ((RCC->CFGR >> ppre1) & 0x3);
        }
    }
    else
    {
        // The position of the PPRE2 bit in RCC->CFGR is different in some stm32
#ifdef _ARCH_CORTEXM3_STM32
        const uint32_t ppre2 = 11;
#elif _ARCH_CORTEXM4_STM32F4
        const uint32_t ppre2 = 13;
#else
#error "Architecture not supported by TimerUtils"
#endif

        if (RCC->CFGR & RCC_CFGR_PPRE2_2)
        {
            inputFrequency /= 1 << ((RCC->CFGR >> ppre2) >> 0x3);
        }
    }

    return inputFrequency;
}

}  // namespace Boardcore
