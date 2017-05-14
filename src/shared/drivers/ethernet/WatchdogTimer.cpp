/* Copyright (c) 2016-2017 Skyward Experimental Rocketry
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
#include "WatchdogTimer.h"

using namespace miosix;
using namespace std;

void __attribute__((naked))  TIM7_IRQHandler() {
    
    saveContext();
    asm volatile("bl _Z8Irq_implv");
    restoreContext(); 
}

void __attribute__((used)) Irq_impl() {
    
    TIM7->SR = 0;    
    auto inst = Singleton< WatchdogTimer >::getInstance();
    inst->stop();
    inst->triggered = true;
    inst->irqCallback();
}


WatchdogTimer::WatchdogTimer() : ms_to_tick(1), triggered(false) {

    {
        FastInterruptDisableLock dLock;
        RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
        RCC_SYNC();
    }
    
    unsigned int busFreq = SystemCoreClock;     //CMSIS defined variable
    if(RCC->CFGR & RCC_CFGR_PPRE1_2) {
        busFreq /= (0x01 << ((RCC->CFGR >> 10) & 0x3));
    }
    
    uint32_t prescaler = busFreq / 1000;    //We want a 1ms/tick timer
    
    // If we obtain a prescaler value upper than 65536 there is a problem:
    // since prescaler value can be at most 65536 the counter will run at a
    // frequency greater than 1kHz. To obtain a timer with a resolution of
    // 1ms we have to determine how many ticks are 1ms. To do this we calculate
    // the timer frequency as busFreq/prescaler and then ms_to_tick as
    // timer_freq/1000
            
    if(prescaler >= 65536) {
        prescaler = 65536;
        unsigned int fclock = busFreq / prescaler;
        ms_to_tick = static_cast< float>(fclock) / 1000.0f;
    }
    
    TIM7->CNT = 0;
    TIM7->ARR = 0;
    TIM7->PSC = prescaler - 1;  // Set prescaler
    TIM7->CR1 |= TIM_CR1_OPM    // Enable one pulse mode
               | TIM_CR1_URS;   // interrupt only on counter overflow
    TIM7->DIER |= TIM_DIER_UIE; // Enable interrupt
    TIM7->EGR |= TIM_EGR_UG;    // Generate update event to preset registers
    
    NVIC_SetPriority(TIM7_IRQn,10);
    NVIC_ClearPendingIRQ(TIM7_IRQn);
    NVIC_EnableIRQ(TIM7_IRQn);
}

WatchdogTimer::~WatchdogTimer() {
    
    stop();
    
    {
        FastInterruptDisableLock dLock;
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM7EN;
        RCC_SYNC();
    }
}

void WatchdogTimer::clear() {
    
    TIM7->CNT = 0;
    triggered = false;
}

void WatchdogTimer::start() {
    
    triggered = false;
    TIM7->CR1 |= TIM_CR1_CEN;    
}

void WatchdogTimer::stop() {
    
    TIM7->CR1 &= ~TIM_CR1_CEN;
}

void WatchdogTimer::setDuration(uint16_t duration) {
    
    float ticks = static_cast< float >(duration) * ms_to_tick;
    TIM7->ARR = static_cast< uint16_t >(ticks + 0.5f);  //round to nearest
}
