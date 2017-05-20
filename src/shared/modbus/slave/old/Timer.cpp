/* Serial driver for Modbus RTU slave device
 *
 * Copyright (c) 2017 Skyward Experimental Rocketry
 * Author: Silvano Seva
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

#include "Timer.h"

using namespace std;
using namespace miosix;

void __attribute__((naked)) TIM1_BRK_TIM9_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z10IRQHandlerv");
    restoreContext();
}

void __attribute__((used)) IRQHandler() {
    
    auto inst = Singleton< Timer >::getInstance();
    
    if(TIM9->SR | TIM_SR_CC1IF)
    {
        TIM9->SR &= ~TIM_SR_CC1IF;
        inst->expFlags[0] = true;
    }
    
    if(TIM9->SR | TIM_SR_CC2IF)
    {
        TIM9->SR &= ~TIM_SR_CC2IF;
        inst->expFlags[1] = true;
    }
}


Timer::Timer() : k(1.0f)
{
    {
        FastInterruptDisableLock dLock;
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
        RCC_SYNC();
    }
    
    TIM9->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE;
    TIM9->ARR = 0xFFFF;
    TIM9->CCMR1 = 0;
    TIM9->CCMR2 = 0;
    TIM9->CNT = 0;
    TIM9->EGR |= TIM_EGR_UG;    //generate update event to load values
    
    NVIC_SetPriority(TIM9_IRQn,15);//Lowest priority for serial
    NVIC_ClearPendingIRQ(TIM9_IRQn);
    NVIC_EnableIRQ(TIM9_IRQn); 
}

Timer::~Timer()
{
    stop();
    {
        FastInterruptDisableLock dLock;
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
        RCC_SYNC();
    }
}

void Timer::start()
{
    TIM9->CR1 |= TIM_CR1_CEN;
}

void Timer::stop()
{
    TIM9->CR1 &= ~TIM_CR1_CEN;
}

void Timer::init(uint32_t refBaud)
{
    uint32_t busFreq = SystemCoreClock;
    if(RCC->CFGR & RCC_CFGR_PPRE2_2)
    {
        busFreq/=1<<(((RCC->CFGR>>13) & 0x3)+1);
    }
    
    /* Here we calculate the prescaler value based on baud rate value.
     * From the modbus RTU specification we have that, if the baud is greater 
     * than 19200, the t1.5 and t3.5 are fixed as the baud is 19200.
     * The timer is set up to accept values multiple of t1.5. Its exact value is
     * given by the formula (1.5 * 11) / baud, since a modbus RTU character is 
     * 11 bit long. Here we use an aproximate one (17/baud instead of 16.5/baud)
     * because we do all the calculations with ints.
     * The prescaler value is given by busFreq * t1.5, that is 
     * (busFreq * 17) / baud. If this value falls above 65536, the maximum 
     * acceptable for the prescaler, the latter is set to 0xFFFF and a suitable
     * value for the correction factor k is computed.
     */
    uint32_t prescaler = (busFreq / min(refBaud,19200)) * 17;
    if(prescaler > 65536)
    {
        TIM9->PSC = 0xFFFF;
        k = (static_cast< float >(busFreq) * 17.0f);
        k /= (65536.0f * static_cast< float >(refBaud));
    }else
    {
        TIM9->PSC = prescaler - 1;
        k = 1.0f;
    }
    
    TIM9->EGR |= TIM_EGR_UG;    //generate update event to load values
}

bool Timer::expired(uint8_t channel)
{
    bool temp = expFlags[channel];
    expFlags[channel] = false;
    return temp;    
}

void Timer::newSetpoint(uint8_t channel, uint8_t ticks)
{
    uint16_t incr = static_cast< uint16_t >((ticks * k) + 0.5f);
    switch(channel)
    {
        case 1:
            TIM9->CCMR1 += incr;
            break;
            
        case 2:
            TIM9->CCMR2 += incr;
            break;
        
        default:
            break;
    }
}

