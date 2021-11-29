/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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
#include <miosix.h>

/**
 * Test developed for the STM32F429 Discovery
 */

using namespace Boardcore::TimerUtils;

void printInputClock(InputClock inputClock);

void testGetTimerInputClock();

void testTimerUtils(TIM_TypeDef *timer);

int main()
{
    printf("TIM1: ");
    testTimerUtils(TIM1);
    printf("\n");

    printf("TIM2: ");
    testTimerUtils(TIM2);
    printf("\n");

    printf("TIM3: ");
    testTimerUtils(TIM3);
    printf("\n");

    printf("TIM4: ");
    testTimerUtils(TIM4);
    printf("\n");

    printf("TIM5: ");
    testTimerUtils(TIM5);
    printf("\n");

    printf("TIM6: ");
    testTimerUtils(TIM6);
    printf("\n");

    printf("TIM7: ");
    testTimerUtils(TIM7);
    printf("\n");

    printf("TIM8: ");
    testTimerUtils(TIM8);
    printf("\n");

    printf("TIM9: ");
    testTimerUtils(TIM9);
    printf("\n");

    printf("TIM10: ");
    testTimerUtils(TIM10);
    printf("\n");

    printf("TIM11: ");
    testTimerUtils(TIM11);
    printf("\n");

    printf("TIM12: ");
    testTimerUtils(TIM12);
    printf("\n");

    printf("TIM13: ");
    testTimerUtils(TIM13);
    printf("\n");

    printf("TIM14: ");
    testTimerUtils(TIM14);
    printf("\n");
}

void printInputClock(InputClock inputClock)
{
    if (inputClock == InputClock::APB1)
    {
        printf("APB1");
    }
    else
    {
        printf("APB2");
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
