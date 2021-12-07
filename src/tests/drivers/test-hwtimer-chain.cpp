/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <Common.h>
#include <drivers/HardwareTimer.h>

using namespace Boardcore;

int main()
{
    {
        miosix::FastInterruptDisableLock dLock;
        // Enable TIM2 + TIM3 + TIM4 peripheral clock
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
    }

    uint32_t prescaler =
        TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1);
    TRACE("Prescaler: %d\n", prescaler);

    HardwareTimer<uint32_t, TimerMode::Chain> timer1(
        TIM3, TIM4, TimerTrigger::ITR2, prescaler);

    HardwareTimer<uint32_t, TimerMode::Single> timer2(TIM2, prescaler);

    timer1.start();
    timer2.start();

    while (true)
    {
        miosix::Thread::sleep(1000);

        uint32_t tick1 = timer1.tick();
        uint32_t tick2 = timer2.tick();

        UNUSED(tick1);
        UNUSED(tick2);

        TRACE("Timer1: %f\n", timer1.toMilliSeconds(tick1));
        TRACE("Timer2: %f\n", timer2.toMilliSeconds(tick1));

        // Delta should remain constant
        TRACE("Delta: %d\n", tick1 - tick2);
    }

    return 0;
}
