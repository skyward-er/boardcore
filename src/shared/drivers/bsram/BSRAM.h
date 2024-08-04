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

#include <miosix.h>
#include <string.h>
#include <utils/ClockUtils.h>

#define PRESERVE __attribute__((section(".preserve")))

/**
 *  Backup SRAM pag 122
 * BKPSRAMEN
 */

namespace Boardcore
{

/**
 * Possible causes for an STM32 reset
 */
enum class ResetReason
{
    RST_LOW_PWR,
    RST_WINDOW_WDG,
    RST_INDEPENDENT_WDG,
    RST_SW,
    RST_POWER_ON,
    RST_PIN,
    RST_UNKNOWN,
};

/**
 * Driver for the STM32F2/F4/F7 backup SRAM, here used as SafeGuard Memory, that
 * is, a memory whose value is preserved across resets.
 *
 * The driver's logic is taken from the stm32_sgm miosix driver made by Matteo
 * Michele Piazzolla.
 */
class BSRAM
{
public:
    BSRAM();

    BSRAM(const BSRAM &)            = delete;
    BSRAM &operator=(const BSRAM &) = delete;

    /**
     * Return the cause of the last reset of the microcontroller
     */
    static ResetReason lastResetReason() { return lastReset; }

protected:
    /**
     * Make the safeguard memory writable again, after a call to disableWrite()
     */
    static void enableWrite();

    /**
     * Temporarily disable writing to the safeguard memory.
     * By deafult, from reset to when the contrsuctor of this class is called
     * the safeguard memory is not writable. After the constructor is called,
     * the safeguard memory is writable.
     */
    static void disableWrite();

    static void readResetRegister();
    static void clearResetFlag();

    static ResetReason lastReset;
};

}  // namespace Boardcore