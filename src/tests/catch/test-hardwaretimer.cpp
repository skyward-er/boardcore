/* Copyright (c) 2019 Skyward Experimental Rocketry
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

// Host is unsupported
#ifndef COMPILE_FOR_HOST

#ifdef STANDALONE_CATCH1_TEST
#include "catch-tests-entry.cpp"
#endif

#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>

#include <catch2/catch.hpp>

using namespace Boardcore;
using namespace miosix;

class TimerTestFixture
{
public:
    TimerTestFixture() : timer32(TIM5), timer16(TIM10)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
    }

    ~TimerTestFixture()
    {
        timer16.disable();
        timer32.disable();

        RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN;
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN;
    }

    GeneralPurposeTimer<uint32_t> timer32;
    GeneralPurposeTimer<uint16_t> timer16;
};

TEST_CASE_METHOD(TimerTestFixture, "Test basic functionality")
{
    timer32.setPrescaler(63);
    timer16.setPrescaler(127);

    REQUIRE(TimerUtils::getMaxDuration(timer32.getTimer()) ==
            Approx(3272.356).margin(0.001));
    REQUIRE(TimerUtils::getMaxDuration(timer16.getTimer()) ==
            Approx(0.049).margin(0.001));

    REQUIRE(TimerUtils::getResolution(timer32.getTimer()) ==
            Approx(0.761).margin(0.001));
    REQUIRE(TimerUtils::getResolution(timer16.getTimer()) ==
            Approx(0.761).margin(0.001));

    REQUIRE(timer32.readCounter() == 0);
    REQUIRE(timer16.readCounter() == 0);

    timer32.enable();
    timer16.enable();

    Thread::sleep(10);

    float timer32timestamp = TimerUtils::toMilliSeconds(timer32.getTimer());
    float timer16timestamp = TimerUtils::toMilliSeconds(timer16.getTimer());

    REQUIRE(timer32timestamp == Approx(10).margin(1));
    REQUIRE(timer16timestamp == Approx(10).margin(1));

    Thread::sleep(30);

    REQUIRE(TimerUtils::toMilliSeconds(timer32.getTimer()) - timer32timestamp ==
            Approx(30).margin(1));
    REQUIRE(TimerUtils::toMilliSeconds(timer16.getTimer()) - timer16timestamp ==
            Approx(30).margin(1));

    timer32.disable();
    timer16.disable();

    timer32timestamp = TimerUtils::toMilliSeconds(timer32.getTimer());
    timer16timestamp = TimerUtils::toMilliSeconds(timer16.getTimer());

    Thread::sleep(20);

    REQUIRE(TimerUtils::toMilliSeconds(timer32.getTimer()) == timer32timestamp);
    REQUIRE(TimerUtils::toMilliSeconds(timer16.getTimer()) == timer16timestamp);
}

TEST_CASE_METHOD(TimerTestFixture, "Test long term precision")
{
    timer32.setPrescaler(63);
    timer16.setPrescaler(65535);  // Max prescaler

    REQUIRE(TimerUtils::getMaxDuration(timer32.getTimer()) ==
            Approx(3272.356).margin(0.001));
    REQUIRE(TimerUtils::getMaxDuration(timer16.getTimer()) ==
            Approx(25.565).margin(0.001));

    REQUIRE(TimerUtils::getResolution(timer32.getTimer()) ==
            Approx(0.761).margin(0.001));
    REQUIRE(TimerUtils::getResolution(timer16.getTimer()) ==
            Approx(390.095).margin(0.001));

    timer32.enable();
    timer16.enable();

    Thread::sleep(24000);

    REQUIRE(TimerUtils::toMilliSeconds(timer32.getTimer()) ==
            Approx(24000).margin(1));
    REQUIRE(TimerUtils::toMilliSeconds(timer16.getTimer()) ==
            Approx(24000).margin(1));

    Thread::sleep(36000);

    REQUIRE(TimerUtils::toMilliSeconds(timer32.getTimer()) ==
            Approx(60000).margin(1));
}

#endif  // COMPILE_FOR_HOST
