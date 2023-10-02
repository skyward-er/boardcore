/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Emilio Corigliano, Alberto Nidasio
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

#include "CountedPWM.h"

namespace Boardcore
{

CountedPWM::CountedPWM(TIM_TypeDef* const pulseTimer,
                       TimerUtils::Channel const pulseChannel,
                       TimerUtils::TriggerSource const pulseTriggerSource,
                       TIM_TypeDef* const counterTimer,
                       TimerUtils::Channel const counterChannel,
                       TimerUtils::TriggerSource const counterTriggerSource)
    : pulseTimer(pulseTimer), pulseChannel(pulseChannel),
      pulseTriggerSource(pulseTriggerSource), counterTimer(counterTimer),
      counterChannel(counterChannel), counterTriggerSource(counterTriggerSource)
{
    // Erase the previous timer configuration
    this->pulseTimer.reset();
    this->counterTimer.reset();

    configureTimers();

    // Keep the timers always enabled. The clock of the pwm timer will be
    // enabled based on the output of the counter timer
    this->pulseTimer.enable();
    this->counterTimer.enable();
}

CountedPWM::~CountedPWM()
{
    pulseTimer.reset();
    counterTimer.reset();
}

void CountedPWM::setFrequency(unsigned int pulseFrequency)
{
    this->pulseFrequency = pulseFrequency;

    if (pulseFrequency == 0)
        return;

    pulseTimer.setFrequency(pulseFrequency * dutyCycleResolution);
    pulseTimer.setAutoReloadRegister(
        TimerUtils::getFrequency(pulseTimer.getTimer()) / pulseFrequency);
}

void CountedPWM::setDutyCycle(float dutyCycle)
{
    if (dutyCycle >= 0 && dutyCycle <= 1)
    {
        this->dutyCycle = dutyCycle;
        pulseTimer.setCaptureCompareRegister(
            pulseChannel,
            static_cast<uint16_t>(
                dutyCycle * pulseTimer.readAutoReloadRegister() + 0.5));
    }
}

void CountedPWM::setDutyCycleResolution(unsigned int dutyCycleResolution)
{
    this->dutyCycleResolution = dutyCycleResolution;
    setFrequency(pulseFrequency);
}

void CountedPWM::generatePulses(uint16_t pulses)
{
    // Reset the counters
    pulseTimer.setCounter(0);
    counterTimer.setCounter(0);

    // Set the capture and compare register to the number of pulses to generate
    counterTimer.setCaptureCompareRegister(counterChannel, pulses);
}

bool CountedPWM::isGenerating()
{
    return counterTimer.readCounter() !=
           counterTimer.readCaptureCompareRegister(counterChannel);
}

void CountedPWM::configureTimers()
{
    // PWM timer
    {
        setFrequency(pulseFrequency);

        // Output/Master: Select TRGO source
        pulseTimer.setMasterMode(masterModeFromChannel(pulseChannel));

        // Input/Slave: Enable the pulseTimer when the output from the
        // counterTimer is high
        pulseTimer.setTriggerSource(pulseTriggerSource);
        pulseTimer.setSlaveMode(TimerUtils::SlaveMode::GATED_MODE);

        // Capture Compare Channel setup in PWM mode
        pulseTimer.setOutputCompareMode(
            pulseChannel, TimerUtils::OutputCompareMode::PWM_MODE_1);
        setDutyCycle(dutyCycle);
        pulseTimer.enableCaptureCompareOutput(pulseChannel);

        // Force the timer to update its configuration
        pulseTimer.generateUpdate();
    }

    // Counter timer
    {
        counterTimer.setAutoReloadRegister(-1);

        // Output/Master: Select TRGO source
        counterTimer.setMasterMode(masterModeFromChannel(counterChannel));

        // Input/Slave: Use the output from counterTimer as clock for pulseTimer
        counterTimer.setTriggerSource(counterTriggerSource);
        counterTimer.setSlaveMode(TimerUtils::SlaveMode::EXTERNAL_CLOCK_MODE_1);

        // Capture Compare Channel setup in PWM mode
        counterTimer.setOutputCompareMode(
            counterChannel, TimerUtils::OutputCompareMode::PWM_MODE_1);
        counterTimer.setCaptureCompareRegister(counterChannel, 0);
        counterTimer.enableCaptureCompareOutput(counterChannel);

        // The output is enabled also for the counterTimer if the user wants to
        // output an enable signal

        // Force the timer to update its configuration
        counterTimer.generateUpdate();
    }
}

}  // namespace Boardcore