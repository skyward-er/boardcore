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
#include "miosix.h"

using namespace std;
using namespace miosix;

//static const unsigned int pwmFreq = 1000; //1kHz
//static const float duty = 0.9; //duty 30%

typedef Gpio<GPIOD_BASE,12> pwmOut; //ch1
typedef Gpio<GPIOD_BASE,13> pwmOut2;  //ch2

Pwm::Pwm(unsigned int fq, float dy)
{
	freq = fq;
	duty = dy;
}

void Pwm::configure(int channel, int mode)
{
	{
        FastInterruptDisableLock dLock;

		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
		RCC_SYNC();
	}


	unsigned int fbus = SystemCoreClock;
	if(RCC->CFGR & RCC_CFGR_PPRE1_2)
	{
		fbus /= 1 << ((RCC->CFGR >> 10) & 0x3);
	}

	switch(mode)
	{
		case 1:
			TIM4->PSC = (fbus / (0xFFFF * freq)) - 1;
			TIM4->ARR = 0xFFFF;
			break;
		case 2:
			static const unsigned int ticksPerPeriod = 10000;
			unsigned int fclock = ticksPerPeriod * freq;

			TIM4->PSC = (fbus / fclock) - 1;
			TIM4->ARR = ticksPerPeriod;
			break;
	}

	switch(channel)
	{
		case 1:
			{
				FastInterruptDisableLock dLock;
				pwmOut::mode(Mode::ALTERNATE);
				pwmOut::alternateFunction(2);
			}
			TIM4->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
			TIM4->CCER |= TIM_CCER_CC1E;
			TIM4->CCR1 = static_cast<uint16_t>(static_cast<float>(TIM4->ARR)*duty);
			break;
		case 2:
			{
				FastInterruptDisableLock dLock;
				pwmOut2::mode(Mode::ALTERNATE);
				pwmOut2::alternateFunction(2);
			}
			TIM4->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
			TIM4->CCER |= TIM_CCER_CC2E;
			TIM4->CCR2 = static_cast<uint16_t>(static_cast<float>(TIM4->ARR)*duty);
			break;
	}

	TIM4->CR1 |= TIM_CR1_ARPE;
}

void Pwm::start()
{
	TIM4->CNT = 0;
	TIM4->EGR |= TIM_EGR_UG;
	TIM4->CR1 |= TIM_CR1_CEN;
}

void Pwm::stop()
{
	TIM4->CCR1=0;
	TIM4->CCR2=0;

	TIM4->CR1 &= ~TIM_CR1_CEN;
}
