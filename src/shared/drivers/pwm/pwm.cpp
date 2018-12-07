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
        if (channels[i].test != 5)
        {
            TRACE("ERROR\n");
        }
    }
}

void PWM::setFrequency(unsigned int frequency) { this->frequency = frequency; }

void PWM::setDutyCycleResolution(unsigned int duty_cycle_resolution)
{
    this->duty_cycle_resolution = duty_cycle_resolution;
}

void PWM::enableChannel(Channel channel, float duty_cycle, Mode mode,
                        Polarity polarity)
{
    int ch                  = static_cast<int>(channel);
    channels[ch].enabled    = true;
    channels[ch].duty_cycle = duty_cycle;
    channels[ch].mode       = mode;
    channels[ch].polarity   = polarity;

    if (started)
    {
        hardwareEnableChannel(channel);
    }
}

void PWM::hardwareEnableChannel(Channel channel)
{
    ChannelConfig cfg = channels[static_cast<int>(channel)];

    // Set duty cycle
    hardawreSetDutyCycle(channel);

    switch (channel)
    {
        case Channel::CH1:
            // Reset the channel 1 bits
            timer.TIM->CCMR1 &= 0xFF00;
            timer.TIM->CCER &= 0xFFF0;

            // Enable preload of CCRx register
            timer.TIM->CCMR1 |= TIM_CCMR1_OC1PE;

            // Set mode
            if (cfg.mode == Mode::MODE_1)
            {
                timer.TIM->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
            }
            else
            {
                timer.TIM->CCMR1 |=
                    TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
            }

            // Set polarity
            if (cfg.polarity == Polarity::ACTIVE_LOW)
            {
                timer.TIM->CCER |= TIM_CCER_CC1P;
            }
            // Enable channel
            timer.TIM->CCER |= TIM_CCER_CC1E;
            break;
        case Channel::CH2:
        // Reset the channel 1 bits
            timer.TIM->CCMR1 &= 0x00FF;
            timer.TIM->CCER &= 0xFF0F;

            // Enable preload of CCRx register
            timer.TIM->CCMR1 |= TIM_CCMR1_OC2PE;

            // Set mode
            if (cfg.mode == Mode::MODE_1)
            {
                timer.TIM->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
            }
            else
            {
                timer.TIM->CCMR1 |=
                    TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;
            }

            // Set polarity
            if (cfg.polarity == Polarity::ACTIVE_LOW)
            {
                timer.TIM->CCER |= TIM_CCER_CC2P;
            }
            // Enable channel
            timer.TIM->CCER |= TIM_CCER_CC2E;
            break;
        case Channel::CH3:
        // Reset the channel 1 bits
            timer.TIM->CCMR2 &= 0xFF00;
            timer.TIM->CCER &= 0xF0FF;

            // Enable preload of CCRx register
            timer.TIM->CCMR2 |= TIM_CCMR2_OC3PE;

            // Set mode
            if (cfg.mode == Mode::MODE_1)
            {
                timer.TIM->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
            }
            else
            {
                timer.TIM->CCMR2 |=
                    TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0;
            }

            // Set polarity
            if (cfg.polarity == Polarity::ACTIVE_LOW)
            {
                timer.TIM->CCER |= TIM_CCER_CC3P;
            }
            // Enable channel
            timer.TIM->CCER |= TIM_CCER_CC3E;
            break;
        case Channel::CH4:
        // Reset the channel 1 bits
            timer.TIM->CCMR2 &= 0x00FF;
            timer.TIM->CCER &= 0x0FFF;

            // Enable preload of CCRx register
            timer.TIM->CCMR2 |= TIM_CCMR2_OC4PE;

            // Set mode
            if (cfg.mode == Mode::MODE_1)
            {
                timer.TIM->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
            }
            else
            {
                timer.TIM->CCMR2 |=
                    TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0;
            }

            // Set polarity
            if (cfg.polarity == Polarity::ACTIVE_LOW)
            {
                timer.TIM->CCER |= TIM_CCER_CC4P;
            }
            // Enable channel
            timer.TIM->CCER |= TIM_CCER_CC4E;
            break;
    }
}

void PWM::setDutyCycle(Channel channel, float duty_cycle)
{
    channels[static_cast<int>(channel)].duty_cycle = duty_cycle;
    if (started && channels[static_cast<int>(channel)].enabled)
    {
        hardawreSetDutyCycle(channel);
    }
}

void PWM::hardawreSetDutyCycle(Channel channel)
{
    float duty_cycle = channels[static_cast<int>(channel)].duty_cycle;
    switch (channel)
    {
        case Channel::CH1:
            timer.TIM->CCR1 =
                static_cast<uint16_t>(duty_cycle * timer.TIM->ARR);
            break;
        case Channel::CH2:
            timer.TIM->CCR2 =
                static_cast<uint16_t>(duty_cycle * timer.TIM->ARR);
            break;
        case Channel::CH3:
            timer.TIM->CCR3 =
                static_cast<uint16_t>(duty_cycle * timer.TIM->ARR);
            break;
        case Channel::CH4:
            timer.TIM->CCR4 =
                static_cast<uint16_t>(duty_cycle * timer.TIM->ARR);
            break;
    }
}

void PWM::disableChannel(Channel channel)
{
    channels[static_cast<int>(channel)].enabled = false;

    if (started)
    {
        hardwareDisableChannel(channel);
    }
}

void PWM::hardwareDisableChannel(Channel channel)
{
    switch (channel)
    {
        case Channel::CH1:
            timer.TIM->CCER &= ~TIM_CCER_CC1E;
            break;
        case Channel::CH2:
            timer.TIM->CCER &= ~TIM_CCER_CC2E;
            break;
        case Channel::CH3:
            timer.TIM->CCER &= ~TIM_CCER_CC3E;
            break;
        case Channel::CH4:
            timer.TIM->CCER &= ~TIM_CCER_CC4E;
            break;
    }
}

void PWM::start()
{
    if (!started)
    {
        {
            FastInterruptDisableLock dLock;

            // Enable timer clock
            *(timer.bus_en_reg) |= timer.TIM_EN;
            RCC_SYNC();
        }

        timer.TIM->CR1 |= TIM_CR1_ARPE;

        uint32_t psc =
            (timer.input_clock_freq / (duty_cycle_resolution * frequency)) - 1;
        if (psc > 0xFFFF)
        {
            psc = 0xFFFF;
        }

        timer.TIM->PSC = psc;
        timer.TIM->ARR = timer.input_clock_freq / ((psc + 1) * frequency);

        timer.TIM->CNT = 0;
        timer.TIM->EGR |= TIM_EGR_UG;

        for (int i = 0; i < 4; i++)
        {
            if (channels[i].enabled)
                hardwareEnableChannel(static_cast<Channel>(i));
        }

        timer.TIM->CR1 |= TIM_CR1_CEN;

        started = true;
    }
}

void PWM::stop()
{
    if (started)
    {
        timer.TIM->CCR1 = 0;
        timer.TIM->CCR2 = 0;

        timer.TIM->CR1 &= ~TIM_CR1_CEN;

        started = false;

        {
            FastInterruptDisableLock dLock;

            // Enable timer clock
            *(timer.bus_en_reg) &= ~timer.TIM_EN;
            RCC_SYNC();
        }
    }
}
