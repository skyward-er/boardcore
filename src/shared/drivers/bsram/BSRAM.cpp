/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include "BSRAM.h"

#include <miosix.h>
#include <string.h>
#include <utils/ClockUtils.h>

namespace Boardcore
{

extern unsigned char _preserve_start asm("_preserve_start");
extern unsigned char _preserve_end asm("_preserve_end");

static unsigned char *preserve_start = &_preserve_start;
static unsigned char *preserve_end   = &_preserve_end;

ResetReason BSRAM::lastReset;

BSRAM::BSRAM()
{
    // Enable PWR clock
    ClockUtils::enablePeripheralClock((void *)PWR_BASE);

    // Enable backup SRAM Clock
    ClockUtils::enablePeripheralClock((void *)BKPSRAM_BASE);

    enableWrite();

    // Enable Backup regulator (Backup SRAM low power Regulator) and wait for
    // its readiness
#ifdef _ARCH_CORTEXM7_STM32F7
    PWR->CSR1 |= PWR_CSR1_BRE;

    while (!(PWR->CSR1 & (PWR_CSR1_BRR)))
        ;
#else
    PWR->CSR |= PWR_CSR_BRE;

    while (!(PWR->CSR & (PWR_CSR_BRR)))
        ;
#endif

    // Retrive last reset reason and clear the pending flag
    readResetRegister();

    // If the reset was caused by low power or an unknown reason, reset all the
    // backup SRAM region.
    if (lastReset == ResetReason::RST_LOW_PWR ||
        lastReset == ResetReason::RST_UNKNOWN)
    {
        memset(preserve_start, 0, preserve_end - preserve_start);
    }
}

void BSRAM::disableWrite()
{
    // Enable Backup Domain write protection
#ifdef _ARCH_CORTEXM7_STM32F7
    PWR->CR1 &= ~PWR_CR1_DBP;
#else
    PWR->CR &= ~PWR_CR_DBP;
#endif
}

void BSRAM::enableWrite()
{
    // Disable Backup Domain write protection
#ifdef _ARCH_CORTEXM7_STM32F7
    PWR->CR1 |= PWR_CR1_DBP;
#else
    PWR->CR |= PWR_CR_DBP;
#endif
}

void BSRAM::readResetRegister()
{
    uint32_t resetReg = RCC->CSR;
    clearResetFlag();
    if (resetReg & RCC_CSR_LPWRRSTF)
    {
        lastReset = ResetReason::RST_LOW_PWR;
    }
    else if (resetReg & RCC_CSR_WWDGRSTF)
    {
        lastReset = ResetReason::RST_WINDOW_WDG;
    }
#ifdef _ARCH_CORTEXM7_STM32F7
    else if (resetReg & RCC_CSR_IWDGRSTF)
#else
    else if (resetReg & RCC_CSR_WDGRSTF)
#endif
    {
        lastReset = ResetReason::RST_INDEPENDENT_WDG;
    }
    else if (resetReg & RCC_CSR_SFTRSTF)
    {
        lastReset = ResetReason::RST_SW;
    }
    else if (resetReg & RCC_CSR_PORRSTF)
    {
        lastReset = ResetReason::RST_POWER_ON;
    }
#ifdef _ARCH_CORTEXM7_STM32F7
    else if (resetReg & RCC_CSR_PINRSTF)
#else
    else if (resetReg & RCC_CSR_PADRSTF)
#endif
    {
        lastReset = ResetReason::RST_PIN;
    }
    else
    {
        lastReset = ResetReason::RST_UNKNOWN;
    }
}

void BSRAM::clearResetFlag() { RCC->CSR |= RCC_CSR_RMVF; }

}  // namespace Boardcore