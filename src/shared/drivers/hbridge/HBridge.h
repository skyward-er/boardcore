/* Copyright (c) 2020 Skyward Experimental Rocketry
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

#pragma once

#include <drivers/timer/PWM.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>

namespace Boardcore
{

/**
 * @brief Driver to operate an H-bridge.
 *
 * From Wikipedia (https://en.wikipedia.org/wiki/H-bridge):
 * An H-bridge is an electronic circuit that switches the polarity of a voltage
 * applied to a load. These circuits are often used in robotics and other
 * applications to allow DC motors to run forwards or backwards. ... In
 * particular, a bipolar stepper motor is almost always driven by a motor
 * controller containing two H bridges.
 *
 * Also, most ICs allow to control the voltage applied to the device by the
 * means of a PWM signal which proportionally controls the voltage.
 *
 * This driver allow to manage the enable pin of the H-bridge and the pwm
 * signal, but it does not still support the control pin needed to reverse the
 * polarity.
 *
 * When disabling the device, the driver waits a predefined amount of time
 * between changing the inhibit pin and the pwm signal. This delay is achieved
 * by making the current thread sleep.
 *
 * Note that the driver assumes that the inhibit pin disables the driver if
 * driven low.
 */
class HBridge
{
public:
    /**
     * @brief Prepares the enable pin and the timer.
     *
     * Note that the timer is enabled automatically when the PWM object is
     * created.
     *
     * @param inhibitPin Inhibit pin of the H-bridge.
     * @param timer Timer peripheral used for the PWM signal.
     * @param channel Timer's channel used for the PWM signal.
     * @param frequency Frequency of the PWM driving the H-bridge.
     * @param dutyCycle Duty cycle of the PWM in the range [0-1].
     * @param disableDelayMs Delay between changing the inhibit pin and the pwm
     * signal when disabling the device.
     */
    HBridge(miosix::GpioPin inhibitPin, TIM_TypeDef* timer,
            TimerUtils::Channel channel, unsigned int frequency,
            float dutyCycle = 0, unsigned int disableDelayMs = 50);

    /**
     * @brief Disables the H-bridge.
     *
     * Note that the timer is disabled automatically when the PWM object is
     * destructed, the PWM signal stops.
     */
    ~HBridge();

    /**
     * @brief Enables the H-bridge and starts the pwm signal.
     */
    void enable();

    /**
     * @brief Stops the pwm signal and deactivates the H-bridge after the
     * configured amount of time.
     */
    void disable();

    bool isEnabled();

    /**
     * @brief Changes the current duty cycle and saves it.
     *
     * Node that the change will take effect immediately and it is not needed to
     * re-enable the H-bridge.
     */
    void setDutyCycle(float dutyCycle);

    /**
     * @brief Same as setDutyCycle() but does not memorize the value.
     *
     * This is useful if you then want to reset the duty cycle to the previously
     * configured value.
     */
    void testDutyCycle(float dutyCycle);

    /**
     * @brief Resets the duty cycle to the last configure value.
     */
    void resetDutyCycle();

private:
    // This class is not copyable!
    HBridge& operator=(const HBridge&) = delete;
    HBridge(const HBridge& hb)         = delete;

    miosix::GpioPin inhibitPin;
    PWM pwm;
    TimerUtils::Channel channel;

    unsigned int frequency;
    float dutyCycle;
    unsigned int disableDelayMs;
};

}  // namespace Boardcore
