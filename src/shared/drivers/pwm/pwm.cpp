/* Copyright (c) 2017-2018 Skyward Experimental Rocketry
 * Authors: Andrea Palumbo
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

PWM::PWM(Timer timer) : timer(timer) {}

void PWM::enable(int channel, unsigned int frequency, bool enable_compl_out,
                 unsigned int duty_cycle_resolution)
{
    {
        FastInterruptDisableLock dLock;

        // Enable timer clock
        *(timer.bus_en_reg) |= timer.TIM_EN;
        RCC_SYNC();
    }

    this->enable_compl_out = enable_compl_out;
    this->channel          = channel;
    this->duty_cycle_res   = duty_cycle_resolution;

	uint32_t psc =  (timer.input_clock_freq / (duty_cycle_resolution * frequency)) - 1;
	if(psc > 0xFFFF)
	{
		psc = 0xFFFF;
	}
    timer.TIM->PSC = psc;
       

    timer.TIM->ARR = timer.input_clock_freq / ((psc - 1) * frequency);

    TRACE("CLK: %d RES: %d, F: %d\nPSC: %d ARR: %d\n", timer.input_clock_freq,
          duty_cycle_resolution, frequency, timer.TIM->PSC, timer.TIM->ARR);
    switch (channel)
    {
        case 0:
            timer.TIM->CCMR1 |=
                TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
            timer.TIM->CCER |= TIM_CCER_CC1E;

            break;
        case 1:
            timer.TIM->CCMR1 |=
                TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
            timer.TIM->CCER |= TIM_CCER_CC2E;
            break;
        case 2:
            timer.TIM->CCMR2 |=
                TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;
            timer.TIM->CCER |= TIM_CCER_CC3E;
            break;
        case 3:
            timer.TIM->CCMR2 |=
                TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;
            timer.TIM->CCER |= TIM_CCER_CC4E;
            break;
    }
	timer.TIM->CR1 |= TIM_CR1_ARPE;
    enabled = true;

    setDutyCycle(duty_cycle);
}

void PWM::setDutyCycle(float duty_cycle)
{
    this->duty_cycle = duty_cycle;

    if (enabled)
    {
        switch (channel)
        {
            case 0:
                timer.TIM->CCR1 =
                    static_cast<uint16_t>(timer.TIM->ARR * duty_cycle);

                break;
            case 1:
                timer.TIM->CCR2 =
                    static_cast<uint16_t>(timer.TIM->ARR * duty_cycle);
                break;
            case 2:
                timer.TIM->CCR2 =
                    static_cast<uint16_t>(timer.TIM->ARR * duty_cycle);
                break;
            case 3:
                timer.TIM->CCR2 =
                    static_cast<uint16_t>(timer.TIM->ARR * duty_cycle);
                break;
        }
    }
}

/*
void PWM::configure(int channel, int mode)
{

    unsigned int fbus = SystemCoreClock;
    if (RCC->CFGR & RCC_CFGR_PPRE1_2)
    {
        fbus /= 1 << ((RCC->CFGR >> 10) & 0x3);
    }

    switch (mode)
    {
        case 1:

            break;
        case 2:
            static const unsigned int ticksPerPeriod = 10000;
            unsigned int fclock                      = ticksPerPeriod * freq;

            timer.TIM->PSC = (fbus / fclock) - 1;
            timer.TIM->ARR = ticksPerPeriod;
            break;
    }

    switch (channel)
    {
        case 1:
        {
            FastInterruptDisableLock dLock;
            pwmOut::mode(Mode::ALTERNATE);
            pwmOut::alternateFunction(2);
        }
            timer.TIM->CCMR1 |=
                TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
            timer.TIM->CCER |= TIM_CCER_CC1E;
            timer.TIM->CCR1 = static_cast<uint16_t>(
                static_cast<float>(timer.TIM->ARR) * duty);
            break;
        case 2:
        {
            FastInterruptDisableLock dLock;
            pwmOut2::mode(Mode::ALTERNATE);
            pwmOut2::alternateFunction(2);
        }
            timer.TIM->CCMR1 |=
                TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
            timer.TIM->CCER |= TIM_CCER_CC2E;
            timer.TIM->CCR2 = static_cast<uint16_t>(
                static_cast<float>(timer.TIM->ARR) * duty);
            break;
    }

    timer.TIM->CR1 |= TIM_CR1_ARPE;
}*/

void PWM::start()
{
    if (enabled)
    {
        timer.TIM->CNT = 0;
        timer.TIM->EGR |= TIM_EGR_UG;
        timer.TIM->CR1 |= TIM_CR1_CEN;
        started = true;
    }
    else
    {
        TRACE("[PWM] Cannot start: PWM not enabled.\n");
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
    }
}

void PWM::disable()
{
    if (enabled)
    {
        if (started)
        {
            stop();
        }

        switch (channel)
        {
            case 0:
                timer.TIM->CCER &= ~TIM_CCER_CC1E;
                break;
            case 1:
                timer.TIM->CCER &= ~TIM_CCER_CC2E;
                break;
            case 2:
                timer.TIM->CCER &= ~TIM_CCER_CC3E;
                break;
            case 3:
                timer.TIM->CCER &= ~TIM_CCER_CC4E;
                break;
        }

        {
            FastInterruptDisableLock dLock;

            // Enable timer clock
            *(timer.bus_en_reg) &= ~timer.TIM_EN;
            RCC_SYNC();
        }
        enabled = false;
    }
}
