/* Copyright (c) 2016-2018 Skyward Experimental Rocketry
 * Author: Matteo Piazzolla
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

#ifndef INTERRUPTMANAGER_H
#define INTERRUPTMANAGER_H

#include "Singleton.h"
#include "miosix.h"
#include <type_traits>

#if ((__GNUC__ < 4) || (__GNUC__ == 4 && __GNUC_MINOR__ < 7) \
|| (__GNUC__ == 4 && __GNUC_MINOR__ == 7 && __GNUC_PATCHLEVEL__ < 3 ))
#error "This file can be compiled with gcc 4.7.3 or higher"
#else

class IGenericInterrupt;

class InterruptManager : public Singleton<InterruptManager>
{
    friend class Singleton<InterruptManager>;

public:
    inline void RegisterInterrupt(IGenericInterrupt* intr, unsigned n){interrupts[n] = intr;}
    static void OnInterruptEvent(unsigned n);

private:
    InterruptManager();
    IGenericInterrupt* interrupts[16];
};

class IGenericInterrupt
{
public:
    virtual void OnReciveInt() = 0;
};

struct EventMode{ enum Mode{
    Rising = 0x01,
    Falling = 0x02,
    RisingAndFalling = Rising | Falling
};};

constexpr unsigned GetInterruptMaskForLine(unsigned N)
{
    return 1 << N;
}

constexpr unsigned GetInterruptFallingMaskForLine(unsigned N)
{
    return 1 << N;
}

constexpr unsigned  GetInterruptRisingMaskForLine(unsigned N)
{
    return 1 << N;
};

constexpr unsigned GetPendingBitForLine(unsigned N)
{
    return 1 << N;
};

constexpr unsigned GetEXTI_IRQn(unsigned N)
{
    return N==0? EXTI0_IRQn :
           N==1? EXTI1_IRQn :
           N==2? EXTI2_IRQn :
           N==3? EXTI3_IRQn :
           N==4? EXTI4_IRQn :
           (N>=5&&N<=9)? EXTI9_5_IRQn :
           EXTI15_10_IRQn;
};

constexpr unsigned GetEXTICR_register_offset(unsigned N)
{
    return N/4;
}

constexpr unsigned ConvertGPIO_BASEtoUnsiged(unsigned P)
{
    return  P == GPIOA_BASE? 0 :
            P == GPIOB_BASE? 1 :
            P == GPIOC_BASE? 2 :
            P == GPIOD_BASE? 3 :
            P == GPIOE_BASE? 4 :
            P == GPIOF_BASE? 5 :
            P == GPIOG_BASE? 6 :
            P == GPIOH_BASE? 7 :
            P == GPIOI_BASE? 8 :
#if (defined (STM32F427xx) || defined (STM32F437xx) || defined (STM32F429xx) ||defined (STM32F439xx))
            P == GPIOJ_BASE? 9 :
            P == GPIOK_BASE? 10 :
#endif                
            0;
};

constexpr unsigned GetEXTICR_register_value(unsigned P, unsigned N)
{
    return (ConvertGPIO_BASEtoUnsiged(P) << ((N%4)*4));
};

template<class _Gpio, int event_mode = EventMode::Rising>
class IInterrupt : public IGenericInterrupt
{
    typedef _Gpio gpio_pin;
    static_assert(event_mode == EventMode::Rising || 
                 event_mode == EventMode::Falling || 
                 event_mode == EventMode::RisingAndFalling, 
            "event_mode template param shoud be member of EventMode struct");
    static_assert(std::is_base_of<miosix::GpioBase, _Gpio>::value, "_Gpio should be a Gpio template class");
    static_assert(  gpio_pin::valueP == GPIOA_BASE ||
                    gpio_pin::valueP == GPIOB_BASE ||
                    gpio_pin::valueP == GPIOC_BASE ||
                    gpio_pin::valueP == GPIOD_BASE ||
                    gpio_pin::valueP == GPIOE_BASE ||
                    gpio_pin::valueP == GPIOF_BASE ||
                    gpio_pin::valueP == GPIOG_BASE ||
                    gpio_pin::valueP == GPIOH_BASE ||
            
#if (defined (STM32F427xx) || defined (STM32F437xx) || defined (STM32F429xx) ||defined (STM32F439xx))
                    gpio_pin::valueP == GPIOI_BASE ||
                    gpio_pin::valueP == GPIOJ_BASE ||
                    gpio_pin::valueP == GPIOK_BASE
#else
                    gpio_pin::valueP == GPIOI_BASE
#endif
            ,"Invalid Gpio port");
    static_assert(gpio_pin::valueN<16, "Invalid Gpio number");    
    
public:    
    IInterrupt()
    {
        if(gpio_pin::valueP != GPIOA_BASE)
        {
            RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
            auto exitcr_reg_value = GetEXTICR_register_value(gpio_pin::valueP, gpio_pin::valueN);
            SYSCFG->EXTICR[GetEXTICR_register_offset(gpio_pin::valueN)] |= exitcr_reg_value;
        }
        gpio_pin::mode(miosix::Mode::INPUT_PULL_DOWN);
        EXTI->IMR |= GetInterruptMaskForLine(gpio_pin::valueN);
        if(event_mode & EventMode::Rising)
            EXTI->RTSR |= GetInterruptRisingMaskForLine(gpio_pin::valueN);
        if(event_mode & EventMode::Falling)
            EXTI->FTSR |= GetInterruptFallingMaskForLine(gpio_pin::valueN);
        NVIC_EnableIRQ(static_cast<IRQn_Type>(GetEXTI_IRQn(gpio_pin::valueN)));
        NVIC_SetPriority(static_cast<IRQn_Type>(GetEXTI_IRQn(gpio_pin::valueN)), 15);
        InterruptManager::GetInstance()->RegisterInterrupt(this,gpio_pin::valueN);
    }
    inline void OnReciveInt() {OnInt(gpio_pin::valueN);}
    virtual void OnInt(unsigned n) = 0;
};
#endif

#endif // INTERRUPTMANAGER_H
