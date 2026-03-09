/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Pietro Bortolus
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

#include <drivers/timer/PWM.h>

#pragma once

namespace Boardcore
{
class SparkPlug
{
public:
    /**
     * @brief Prepare the timer.
     *
     * Note that the timer peripheral's clock is enabled automatically when the
     * PWM object is created.
     *
     * @param timer Timer peripheral used for the PWM signal.
     * @param pwmChannel Timer's channel used for the PWM signal.
     */
    explicit SparkPlug(TIM_TypeDef* const timer, uint16_t frequency,
                       TimerUtils::Channel pwmChannel)
        : pwm(timer, frequency), pwmChannel(pwmChannel)
    {
        stop();
    };

    /**
     * @brief Starts producing the PWM signal.
     */
    void enable() { pwm.enableChannel(pwmChannel); };

    /**
     * @brief Stops producing the PWM signal.
     */
    void disable() { pwm.disableChannel(pwmChannel); };

    /**
     * @brief Starts the sparking
     */
    void start() { pwm.setDutyCycle(pwmChannel, 0.5f); };

    /**
     * @brief Stops the sparking
     */
    void stop() { pwm.setDutyCycle(pwmChannel, 0.0f); };

private:
    // This class is not copyable!
    SparkPlug& operator=(const SparkPlug&) = delete;
    SparkPlug(const SparkPlug& s)          = delete;

    PWM pwm;
    TimerUtils::Channel pwmChannel;
};
}  // namespace Boardcore

