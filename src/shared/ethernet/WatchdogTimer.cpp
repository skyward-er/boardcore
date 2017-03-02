/*
 * Simple watchdog timer with 1ms resolution that calls a function when expires
 * Copyright (c) 2017 Skyward Experimental Rocketry
 * Author: Silvano Seva
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "WatchdogTimer.h"

using namespace miosix;
using namespace std;

void __attribute__((naked))  TIM7_IRQHandler() {
    
    saveContext();
    asm volatile("bl _Z8Irq_implv");
    restoreContext(); 
}

void Irq_impl() {
    
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