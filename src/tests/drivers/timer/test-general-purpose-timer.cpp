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

#include <drivers/timer/GeneralPurposeTimer.h>
#include <drivers/timer/TimerUtils.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;

GeneralPurposeTimer<uint16_t> timer(TIM4);
GeneralPurposeTimer<uint16_t>::Channel channel =
    GeneralPurposeTimer<uint16_t>::Channel::CHANNEL_1;
constexpr int frequency = 123456;
GpioPin timerPin        = GpioPin(GPIOB_BASE, 7);

int main()
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    timerPin.mode(Mode::ALTERNATE);
    timerPin.alternateFunction(2);

    timer.reset();
    timer.setPrescaler(
        TimerUtils::computePrescalerValue(timer.getTimer(), frequency * 4));
    timer.setAutoReloadRegister(100);
    timer.setOutputCompareMode(
        GeneralPurposeTimer<uint16_t>::OutputCompareMode::TOGGLE, channel);
    timer.generateUpdate();
    timer.setCaptureCompareRegister(100, channel);
    timer.enableCaptureCompareOutput(channel);
    timer.enableCaptureCompareComplementaryOutput(channel);
    timer.enable();

    while (true)
        delayMs(1000);
}