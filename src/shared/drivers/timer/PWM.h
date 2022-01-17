/* Copyright (c) 2017-2021 Skyward Experimental Rocketry
 * Authors: Andrea Palumbo, Luca Erbetta, Alberto Nidasio
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

#pragma once

#include <drivers/timer/GeneralPurposeTimer.h>

namespace Boardcore
{

/**
 * @brief Driver for easy access to the PWM capabilities of the general purpose
 * timers.
 *
 * Note that the pwm timer is started when the object is created (and disabled
 * when deleted) but no channels are enalbed. This means that you will only need
 * to use the channels whithout worring about the underlying timer.
 *
 * The PWM driver accepts a the pointer to the peripheral registers of a timer
 * and uses it as a 16bit general purpose timer. No checks are in place to this
 * pointer, thus make shure to pass a proper value!
 *
 * Moreover, even 32bit general purpose timers are used as if they where 16bit.
 * At the moment there is no need for further accuracy but if it ever will be,
 * exending this class is simble, just add a template parameter and pass it to
 * the GeneralPurposeTimer parameter.
 *
 * Check out the following spread sheet to visually see how the timers registers
 * are calculated and the accuracy achieved:
 * https://docs.google.com/spreadsheets/d/1FiNDVU7Rg98yZzz1dZ4GDAq3-nEg994ziezCawJ-OK4/edit#gid=0
 */
class PWM
{
public:
    enum class Polarity : uint16_t
    {
        NORMAL   = 0,   ///< Signal is high for the duty cycle time.
        REVERSED = 0x1  ///< Signal is low for the duty cycle time.
    };

    /**
     * @brief Sets up the PWM timer.
     *
     * @param timer Pointer to the timer's peripheral registers.
     * @param pwmFrequency Frequency of the PWM signal.
     * @param dutyCycleResolution Duty cycle levels.
     */
    explicit PWM(TIM_TypeDef *timer, unsigned int pwmFrequency = 50,
                 unsigned int dutyCycleResolution = 1000);

    ~PWM();

    void setFrequency(unsigned int pwmFrequency);

    void setDutyCycleResolution(unsigned int dutyCycleResolution);

    void enableChannel(TimerUtils::Channel channel,
                       Polarity polarity = Polarity::NORMAL);

    void disableChannel(TimerUtils::Channel channel);

    bool isChannelEnabled(TimerUtils::Channel channel);

    /**
     * @brief Sets the duty cycle for the specified channel.
     *
     * Changes the duty cycle only if the specified value is in the range [0,1].
     *
     * @param channel Channel to change the duty cycle
     * @param dutyCycle Relative duty cycle, ranges from 0 to 1
     */
    void setDutyCycle(TimerUtils::Channel channel, float dutyCycle);

private:
    void setTimerConfiguration();

    GP16bitTimer timer;
    unsigned int pwmFrequency;
    unsigned int dutyCycleResolution;
};

}  // namespace Boardcore
