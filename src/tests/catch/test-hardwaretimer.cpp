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

#ifdef STANDALONE_CATCH1_TEST
#include "catch-tests-entry.cpp"
#endif

#include <drivers/HardwareTimer.h>
#include <miosix.h>

#include <catch2/catch.hpp>

using namespace Boardcore;
using namespace miosix;

class TimerTestFixture
{
public:
    TimerTestFixture()
        : timer32(TIM5, TimerUtils::getPrescalerInputFrequency(
                            TimerUtils::InputClock::APB1)),
          timer16(TIM10, TimerUtils::getPrescalerInputFrequency(
                             TimerUtils::InputClock::APB2))
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
    }

    ~TimerTestFixture()
    {
        timer16.stop();
        timer32.stop();

        RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN;
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN;
    }

    HardwareTimer<uint32_t> timer32;
    HardwareTimer<uint16_t> timer16;
};

TEST_CASE_METHOD(TimerTestFixture, "Test basic functionality")
{
    timer32.setPrescaler(63);
    timer16.setPrescaler(127);

    REQUIRE(timer32.getMaxDuration() == Approx(3272.356).margin(0.001));
    REQUIRE(timer16.getMaxDuration() == Approx(0.049).margin(0.001));

    REQUIRE(timer32.getResolution() == Approx(0.761).margin(0.001));
    REQUIRE(timer16.getResolution() == Approx(0.761).margin(0.001));

    REQUIRE(timer32.tick() == 0);
    REQUIRE(timer16.tick() == 0);

    REQUIRE(timer32.start() == 0);
    REQUIRE(timer16.start() == 0);

    Thread::sleep(10);

    uint32_t tick32 = timer32.tick();
    uint32_t tick16 = timer16.tick();

    REQUIRE(timer32.toMilliSeconds(tick32) == Approx(10).margin(1));
    REQUIRE(timer16.toMilliSeconds(tick16) == Approx(10).margin(1));

    Thread::sleep(30);

    REQUIRE(timer32.toMilliSeconds(timer32.tick() - tick32) ==
            Approx(30).margin(1));
    REQUIRE(timer16.toMilliSeconds(timer16.tick() - tick16) ==
            Approx(30).margin(1));

    tick32 = timer32.stop();
    tick16 = timer16.stop();

    Thread::sleep(20);

    REQUIRE(timer32.tick() == tick32);
    REQUIRE(timer16.tick() == tick16);
}

TEST_CASE_METHOD(TimerTestFixture, "Test long term precision")
{
    timer32.setPrescaler(63);
    timer16.setPrescaler(65535);  // Max prescaler

    REQUIRE(timer32.getMaxDuration() == Approx(3272.356).margin(0.001));
    REQUIRE(timer16.getMaxDuration() == Approx(25.565).margin(0.001));

    REQUIRE(timer32.getResolution() == Approx(0.761).margin(0.001));
    REQUIRE(timer16.getResolution() == Approx(390.095).margin(0.001));

    REQUIRE(timer32.start() == 0);
    REQUIRE(timer16.start() == 0);

    Thread::sleep(24000);

    timer32.tick();
    timer16.tick();

    REQUIRE(timer32.toMilliSeconds(timer32.tick()) == Approx(24000).margin(1));
    REQUIRE(timer16.toMilliSeconds(timer16.tick()) == Approx(24000).margin(1));

    Thread::sleep(36000);

    REQUIRE(timer32.toMilliSeconds(timer32.tick()) == Approx(60000).margin(1));
}
