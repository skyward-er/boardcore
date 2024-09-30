/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Nicolò Caruso
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

#include <chrono>

namespace Boardcore
{

/**
 * @brief Dip switch driver to read the current status of the switch
 */
class DipSwitch
{
public:
    DipSwitch(miosix::GpioPin& sh, miosix::GpioPin& clk, miosix::GpioPin& qh,
              std::chrono::microseconds microSecClk)
        : sh{sh}, clk{clk}, qh{qh}, microSecClk{microSecClk.count()}
    {
    }

    /**
     * @brief Performs a read of the dipSwitch
     *
     * @return uint8_t the read of the dip switch in big endian order. Dip 1 as
     * most significant value, dip 8 as less significant.
     * @note The order may depend on the dipswitch
     * @note visually the order in a 8bit word is:
     * dip order:  [ 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 ]
     * bits order: 12345678
     */
    uint8_t read();

private:
    miosix::GpioPin& sh;   ///< Shift pin / not load
    miosix::GpioPin& clk;  ///< Clock pin
    miosix::GpioPin& qh;   ///< Data output pin
    int64_t microSecClk;

    /**
     * @brief Reads a bit from the dip switch shift register
     */
    uint8_t readBit();
};
}  // namespace Boardcore
