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

#include <drivers/DipSwitch/DipSwitch.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>

using namespace miosix;

/**
 * The event counter is subscribed to topic 5.
 * Try to post events to that topic and check the counter.
 */
int main()
{
    uint32_t microSecClk = 100;
    GpioPin sh           = dipSwitch::sh::getPin();
    GpioPin clk          = dipSwitch::clk::getPin();
    GpioPin qh           = dipSwitch::qh::getPin();

    DipSwitch dipSwitch(sh, clk, qh, microSecClk);
    while (true)
    {
        DipStatus status;
        Thread::sleep(1000);
        status = dipSwitch.read();
        printf(
            "Read from dipSwitch: isArp %d | hasBackup %d | uint ip config: "
            "%d\n",
            status.isARP, status.hasBackup, status.ipConfig);
    }
    return 0;
}
