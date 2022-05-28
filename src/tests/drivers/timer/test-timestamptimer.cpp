/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Conterio, Alberto Nidasio
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

#include <drivers/timer/TimerUtils.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;
using namespace TimerUtils;
using namespace ClockUtils;

void printInputClock(APB bus);

void testTimerUtils(TIM_TypeDef *timer);

int main()
{
    testTimerUtils(TIM2);

    printf("Initialization should be complete\n");

    delayMs(1000);

    for (int i = 0; i < 10; i++)
    {
        long long prevTick = getTick();

        uint64_t timestamp = TimestampTimer::getTimestamp();

        // cppcheck-suppress invalidPrintfArgType_uint
        printf("%12llu us, %12.3f ms, %12.6f s, %12lld tick \n", timestamp,
               timestamp / 1e3, timestamp / 1e6, prevTick);

        Thread::sleepUntil(prevTick + 1000);
    }

    printf("Now resetting the TimestampTimer\n");

    TimestampTimer::resetTimestamp();

    while (true)
    {
        long long prevTick = getTick();

        uint64_t timestamp = TimestampTimer::getTimestamp();

        // cppcheck-suppress invalidPrintfArgType_uint
        printf("%12llu us, %12.3f ms, %12.6f s, %12lld tick \n", timestamp,
               timestamp / 1e3, timestamp / 1e6, prevTick);

        Thread::sleepUntil(prevTick + 1000);
    }
}

void printInputClock(APB bus)
{
    if (bus == APB::APB1)
    {
        printf("Timer input clock from APB1\n");
    }
    else
    {
        printf("Timer input clock from APB2\n");
    }
}

void testTimerUtils(TIM_TypeDef *timer)
{
    // Print the timer clock source
    printInputClock(getTimerInputClock(timer));

    // Print the timer input clock frequency
    printf("Prescaler input frequency: %ld\n",
           getPrescalerInputFrequency(timer));

    // Print the timer frequency
    printf("Timer frequency: %ld\n", getFrequency(timer));

    // Time conversion
    printf("Timer counter in us: %f\n", toMicroSeconds(timer));
    printf("Timer counter in us: %lld\n", toIntMicroSeconds(timer));
    printf("Timer counter in ms: %f\n", toMilliSeconds(timer));
    printf("Timer counter in s: %f\n", toSeconds(timer));

    // Timer resolution
    printf("Timer resulution in us: %f\n", getResolution(timer));

    // Timer max duration
    printf("Timer max duration in s: %f\n", getMaxDuration(timer));
}
