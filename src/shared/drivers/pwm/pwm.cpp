/* Copyright (c) 2017-2018 Skyward Experimental Rocketry
 * Authors: Andrea Palumbo, Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "pwm.h"
#include "Common.h"

using namespace std;
using namespace miosix;

PWM::PWM(Timer timer, unsigned int frequency,
         unsigned int duty_cycle_resolution)
    : timer(timer), frequency(frequency),
      duty_cycle_resolution(duty_cycle_resolution)
{
    for (int i = 0; i < 4; i++)
    {
        channels[i].channel = static_cast<PWMChannel>(i);
    }

    // Enable clock on the Timer
    {
        FastInterruptDisableLock dLock;

        // Enable timer clock
        *(timer.bus_en_reg) |= timer.TIM_EN;
        RCC_SYNC();
    }

    timer.TIM->CR1 |= TIM_CR1_ARPE;  // Auto reload register preload

    hardwareUpdateRegisters();
}

PWM::~PWM()
{
    // Disable clock on timer
    {
        FastInterruptDisableLock dLock;

        // Enable timer clock
        *(timer.bus_en_reg) &= ~timer.TIM_EN;
        RCC_SYNC();
    }
}

void PWM::setFrequency(unsigned int frequency) { this->frequency = frequency; 
hardwareUpdateRegisters();}

void PWM::setDutyCycleResolution(unsigned int duty_cycle_resolution)
{
    this->duty_cycle_resolution = duty_cycle_resolution;
    hardwareUpdateRegisters();
}

void PWM::enableChannel(PWMChannel channel, float duty_cycle, PWMMode mode,
                        PWMPolarity polarity)
{
    int ch                  = static_cast<int>(channel);
    channels[ch].enabled    = true;
    channels[ch].duty_cycle = duty_cycle;
    channels[ch].mode       = mode;
    channels[ch].polarity   = polarity;

    hardwareEnableChannel(channel);
}

void PWM::hardwareEnableChannel(PWMChannel channel)
{
    PWMChannelConfig cfg = channels[static_cast<int>(channel)];

    // Set duty cycle
    hardwareSetDutyCycle(channel);

    switch (channel)
    {
        case PWMChannel::CH1:
            // Reset the channel 1 bits
            timer.TIM->CCMR1 &= 0xFF00;
            timer.TIM->CCER &= 0xFFF0;

            // Enable preload of CCRx register
            timer.TIM->CCMR1 |= TIM_CCMR1_OC1PE;

            // Set mode
            if (cfg.mode == PWMMode::MODE_1)
            {
                timer.TIM->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
            }
            else
            {
                timer.TIM->CCMR1 |=
                    TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
            }

            // Set polarity
            if (cfg.polarity == PWMPolarity::ACTIVE_LOW)
            {
                timer.TIM->CCER |= TIM_CCER_CC1P;
            }
            // Enable channel
            timer.TIM->CCER |= TIM_CCER_CC1E;
            break;
        case PWMChannel::CH2:
            // Reset the channel 1 bits
            timer.TIM->CCMR1 &= 0x00FF;
            timer.TIM->CCER &= 0xFF0F;

            // Enable preload of CCRx register
            timer.TIM->CCMR1 |= TIM_CCMR1_OC2PE;

            // Set mode
            if (cfg.mode == PWMMode::MODE_1)
            {
                timer.TIM->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
            }
            else
            {
                timer.TIM->CCMR1 |=
                    TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;
            }

            // Set polarity
            if (cfg.polarity == PWMPolarity::ACTIVE_LOW)
            {
                timer.TIM->CCER |= TIM_CCER_CC2P;
            }
            // Enable channel
            timer.TIM->CCER |= TIM_CCER_CC2E;
            break;
        case PWMChannel::CH3:
            // Reset the channel 1 bits
            timer.TIM->CCMR2 &= 0xFF00;
            timer.TIM->CCER &= 0xF0FF;

            // Enable preload of CCRx register
            timer.TIM->CCMR2 |= TIM_CCMR2_OC3PE;

            // Set mode
            if (cfg.mode == PWMMode::MODE_1)
            {
                timer.TIM->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
            }
            else
            {
                timer.TIM->CCMR2 |=
                    TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0;
            }

            // Set polarity
            if (cfg.polarity == PWMPolarity::ACTIVE_LOW)
            {
                timer.TIM->CCER |= TIM_CCER_CC3P;
            }
            // Enable channel
            timer.TIM->CCER |= TIM_CCER_CC3E;
            break;
        case PWMChannel::CH4:
            // Reset the channel 1 bits
            timer.TIM->CCMR2 &= 0x00FF;
            timer.TIM->CCER &= 0x0FFF;

            // Enable preload of CCRx register
            timer.TIM->CCMR2 |= TIM_CCMR2_OC4PE;

            // Set mode
            if (cfg.mode == PWMMode::MODE_1)
            {
                timer.TIM->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
            }
            else
            {
                timer.TIM->CCMR2 |=
                    TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0;
            }

            // Set polarity
            if (cfg.polarity == PWMPolarity::ACTIVE_LOW)
            {
                timer.TIM->CCER |= TIM_CCER_CC4P;
            }
            // Enable channel
            timer.TIM->CCER |= TIM_CCER_CC4E;
            break;
    }
}

void PWM::setDutyCycle(PWMChannel channel, float duty_cycle)
{
    channels[static_cast<int>(channel)].duty_cycle = duty_cycle;

    hardwareSetDutyCycle(channel);
}

void PWM::hardwareSetDutyCycle(PWMChannel channel)
{
    float duty_cycle = channels[static_cast<int>(channel)].duty_cycle;
    switch (channel)
    {
        case PWMChannel::CH1:
            timer.TIM->CCR1 =
                static_cast<uint16_t>(duty_cycle * timer.TIM->ARR);
            break;
        case PWMChannel::CH2:
            timer.TIM->CCR2 =
                static_cast<uint16_t>(duty_cycle * timer.TIM->ARR);
            break;
        case PWMChannel::CH3:
            timer.TIM->CCR3 =
                static_cast<uint16_t>(duty_cycle * timer.TIM->ARR);
            break;
        case PWMChannel::CH4:
            timer.TIM->CCR4 =
                static_cast<uint16_t>(duty_cycle * timer.TIM->ARR);
            break;
    }
}

void PWM::disableChannel(PWMChannel channel)
{
    channels[static_cast<int>(channel)].enabled = false;
    hardwareDisableChannel(channel);
}

void PWM::hardwareDisableChannel(PWMChannel channel)
{
    switch (channel)
    {
        case PWMChannel::CH1:
            timer.TIM->CCER &= ~TIM_CCER_CC1E;
            break;
        case PWMChannel::CH2:
            timer.TIM->CCER &= ~TIM_CCER_CC2E;
            break;
        case PWMChannel::CH3:
            timer.TIM->CCER &= ~TIM_CCER_CC3E;
            break;
        case PWMChannel::CH4:
            timer.TIM->CCER &= ~TIM_CCER_CC4E;
            break;
    }
}

void PWM::hardwareUpdateRegisters()
{
    uint32_t psc =
        (timer.input_clock_freq / (duty_cycle_resolution * frequency)) - 1;
    if (psc > 0xFFFF)
    {
        psc = 0xFFFF;
    }

    timer.TIM->PSC = psc;
    timer.TIM->ARR = timer.input_clock_freq / ((psc + 1) * frequency);
}

void PWM::start()
{
    if (!started)
    {
        timer.TIM->CNT = 0;
        timer.TIM->EGR |= TIM_EGR_UG;

        timer.TIM->CR1 |= TIM_CR1_CEN;

        // Advanced-control timers need Main Output Enable bit to 1 to output pwm
        if(timer.TIM == TIM1 || timer.TIM == TIM8)
        {
            timer.TIM->BDTR |= TIM_BDTR_MOE;
        }

        started = true;
    }
}

void PWM::stop()
{
    if (started)
    {
        timer.TIM->CR1 &= ~TIM_CR1_CEN;

        if(timer.TIM == TIM1 || timer.TIM == TIM8)
        {
            timer.TIM->BDTR &= ~TIM_BDTR_MOE;
        }

        started = false;
    }
}
