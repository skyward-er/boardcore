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

#include "HBridge.h"

HBridge::HBridge(miosix::GpioPin inhibit, PWM::Timer timer, PWMChannel channel,
                 uint32_t frequency, float duty_cycle,
                 uint16_t disable_delay_ms)
    : pin_inh(inhibit), pwm(timer, frequency), channel(channel),
      duty_cycle(duty_cycle), disable_delay_ms(disable_delay_ms)
{
    pin_inh.low();

    status.timestamp = timer::TimestampTimer::getTimestamp();
    status.state     = HBridgeState::DISABLED;

    // Start PWM with 0 duty cycle to keep IN pins low
    pwm.enableChannel(channel, 0.0f);

    pwm.start();
}

HBridge::~HBridge()
{
    disable();

    pwm.stop();
}

void HBridge::disable()
{
    if (status.state == HBridgeState::ENABLED)
    {
        pwm.setDutyCycle(channel, 0.0f);  // Set duty cycle to 0 to leave the IN
                                          // pin of the h-bridge low

        Thread::sleep(disable_delay_ms);  // Wait a short delay

        pin_inh.low();  // Disable h-bridge

        status.timestamp = timer::TimestampTimer::getTimestamp();
        status.state     = HBridgeState::DISABLED;
    }
}

void HBridge::enable() { enableHBridge(channel, pin_inh, duty_cycle); }

void HBridge::enableTest(float test_duty_cycle)
{
    enableHBridge(channel, pin_inh, test_duty_cycle);
}

HBridgeStatus HBridge::getStatus() { return status; }

void HBridge::enableHBridge(PWMChannel channel, GpioPin& inh, float duty_cycle)
{
    if (status.state == HBridgeState::DISABLED)
    {
        // Enable PWM Generation
        pwm.setDutyCycle(channel, duty_cycle);
        // enable h-bridge
        inh.high();

        status.timestamp = timer::TimestampTimer::getTimestamp();
        status.state     = HBridgeState::ENABLED;
    }
}
