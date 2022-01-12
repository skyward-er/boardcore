/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#include <drivers/pwm/pwm.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>

#include "HBridgeData.h"

using miosix::GpioPin;
using miosix::Thread;

namespace Boardcore
{

/**
 * @brief Interface class to operate an h-bridge.
 *
 * Provides the ability to enable an h-bridge using a "nominal" duty cycle or
 * using a "test" duty cycle.
 */
class HBridge
{
public:
    /**
     * @brief Create a new HBridge
     *
     * @param   inhibit            Inhibit pin of the h-bridge
     * @param   timer              Timer peripheral used for the PWM signal
     * @param   channel            PWM channel
     * @param   frequency          Frequency of the PWM driving the h-bridge
     * @param   dutyCycle         Duty cycle of the PWM (from 0.0 to 1.0)
     * @param   disableDelayMs   Period of time where the IN must be kept
     *                             low before bringing ENA/INH low
     */
    HBridge(GpioPin inhibit, PWM::Timer timer, PWMChannel channel,
            uint32_t frequency, float dutyCycle, uint16_t disableDelayMs = 50);

    ~HBridge();

    /**
     * @brief Deactivates the h-bridge
     */
    void disable();

    /**
     * @brief Enables the h-bridge
     *
     * call disable() to stop
     */
    void enable();

    /**
     * @brief Enables the h-bridge using the "test" duty cycle.
     *        Call disable() to stop
     *
     * @param testDutyCycle   Duty cycle to be used when testing the
     *                          h-bridge for continuity
     */
    void enableTest(float testDutyCycle);

    /**
     * @return the HBridge object status
     */
    HBridgeStatus getStatus();

private:
    /**
     * @brief Enables the h-bridge.
     *
     * @param channel       the h-bridge PWM channel
     * @param enablePin       enable pin of the h-bridge
     * @param dutyCycle    the duty cycle to be used
     */
    void enableHBridge(PWMChannel channel, GpioPin& enablePin, float dutyCycle);

    HBridge(const HBridge& hb) = delete;

    GpioPin pinInterrupt;
    PWM pwm;
    PWMChannel channel;

    float dutyCycle;
    uint16_t disableDelayMs;

    HBridgeStatus status;
};

}  // namespace Boardcore
