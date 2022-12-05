/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio, Luca Erbetta
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

#pragma once

#include <interfaces/arch_registers.h>
#include <kernel/kernel.h>

namespace Boardcore
{

namespace ClockUtils
{

/**
 * @brief Timer input clock.
 */
enum class APB
{
    APB1,
    APB2
};

/**
 * @brief Computes the output clock frequency for peripherals on the given APB.
 *
 * @param bus Advanced Peripheral Bus
 * @return Clock frequency of peripherals.
 */
uint32_t getAPBPeripheralsClock(APB bus);

/**
 * @brief Computes the output clock frequency for timers on the given APB.
 *
 * @param bus Advanced Peripheral Bus
 * @return Clock frequency of timers.
 */
uint32_t getAPBTimersClock(APB bus);

/**
 * @brief Enables a peripheral clock source from the APB1 and APB2 peripheral
 * buses.
 */
bool enablePeripheralClock(void* peripheral);

/**
 * @brief Disables a peripheral clock source from the APB1 and APB2 peripheral
 * buses.
 */
bool disablePeripheralClock(void* peripheral);

}  // namespace ClockUtils

inline uint32_t ClockUtils::getAPBPeripheralsClock(APB bus)
{
    // The global variable SystemCoreClock from ARM's CMIS allows to know the
    // CPU frequency.
    uint32_t inputFrequency = SystemCoreClock;

    // The APB frequency may be a submultiple of the CPU frequency, due to the
    // bus at which the peripheral is connected being slower.
    // The RCC->CFGR register tells us how slower the APB bus is running.
    if (bus == APB::APB1)
    {
        // The position of the PPRE1 bit in RCC->CFGR is different in some stm32
#ifdef _ARCH_CORTEXM3_STM32
        const uint32_t ppre1 = 8;
#elif _ARCH_CORTEXM4_STM32F4 | _ARCH_CORTEXM3_STM32F2
        const uint32_t ppre1 = 10;
#else
#error "Architecture not supported by TimerUtils"
#endif

        if (RCC->CFGR & RCC_CFGR_PPRE1_2)
        {
            inputFrequency /= 2 << ((RCC->CFGR >> ppre1) & 0b11);
        }
    }
    else
    {
        // The position of the PPRE2 bit in RCC->CFGR is different in some stm32
#ifdef _ARCH_CORTEXM3_STM32
        const uint32_t ppre2 = 11;
#elif _ARCH_CORTEXM4_STM32F4 | _ARCH_CORTEXM3_STM32F2
        const uint32_t ppre2 = 13;
#else
#error "Architecture not supported by TimerUtils"
#endif

        if (RCC->CFGR & RCC_CFGR_PPRE2_2)
        {
            inputFrequency /= 2 << ((RCC->CFGR >> ppre2) & 0b11);
        }
    }

    return inputFrequency;
}

inline uint32_t ClockUtils::getAPBTimersClock(APB bus)
{
    // The global variable SystemCoreClock from ARM's CMIS allows to know the
    // CPU frequency.
    uint32_t inputFrequency = SystemCoreClock;

    // The APB frequency may be a submultiple of the CPU frequency, due to the
    // bus at which the peripheral is connected being slower.
    // The RCC->CFGR register tells us how slower the APB bus is running.
    if (bus == APB::APB1)
    {
        // The position of the PPRE1 bit in RCC->CFGR is different in some stm32
#ifdef _ARCH_CORTEXM3_STM32
        const uint32_t ppre1 = 8;
#elif _ARCH_CORTEXM4_STM32F4 | _ARCH_CORTEXM3_STM32F2
        const uint32_t ppre1 = 10;
#else
#error "Architecture not supported by TimerUtils"
#endif

        if (RCC->CFGR & RCC_CFGR_PPRE1_2)
        {
            inputFrequency /= 1 << ((RCC->CFGR >> ppre1) & 0b11);
        }
    }
    else
    {
        // The position of the PPRE2 bit in RCC->CFGR is different in some stm32
#ifdef _ARCH_CORTEXM3_STM32
        const uint32_t ppre2 = 11;
#elif _ARCH_CORTEXM4_STM32F4 | _ARCH_CORTEXM3_STM32F2
        const uint32_t ppre2 = 13;
#else
#error "Architecture not supported by TimerUtils"
#endif

        if (RCC->CFGR & RCC_CFGR_PPRE2_2)
        {
            inputFrequency /= 1 << ((RCC->CFGR >> ppre2) & 0b11);
        }
    }

    return inputFrequency;
}

inline bool ClockUtils::enablePeripheralClock(void* peripheral)
{
    miosix::FastInterruptDisableLock dLock;

    switch (reinterpret_cast<uint32_t>(peripheral))
    {
        // AHB1 peripherals
        {
            case GPIOA_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
                break;
            case GPIOB_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
                break;
            case GPIOC_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
                break;
            case GPIOD_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
                break;
            case GPIOE_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
                break;
            case GPIOF_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
                break;
            case GPIOG_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
                break;
            case GPIOH_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
                break;
            case GPIOI_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
                break;
#ifdef STM32F429xx
            case GPIOJ_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOJEN;
                break;
            case GPIOK_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOKEN;
                break;
#endif
            case CRC_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
                break;
            case BKPSRAM_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
                break;
#ifndef _ARCH_CORTEXM3_STM32F2
            case CCMDATARAM_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_CCMDATARAMEN;
                break;
#endif
            case DMA1_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
                break;
            case DMA2_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
                break;
#ifdef STM32F429xx
            case DMA2D_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_DMA2DEN;
                break;
#endif
#ifndef _ARCH_CORTEXM3_STM32F2
            case ETH_MAC_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACEN;
                break;
#endif
            case USB_OTG_HS_PERIPH_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_OTGHSEN;
                break;
        }

        // AHB2 peripherals
        {
#ifndef _ARCH_CORTEXM3_STM32F2
            case DCMI_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN;
                break;
#endif
            case RNG_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
                break;
            case USB_OTG_FS_PERIPH_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
                break;
        }

        // APB1 peripherals
        {
            case TIM2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
                break;
            case TIM3_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
                break;
            case TIM4_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
                break;
            case TIM5_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
                break;
            case TIM6_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
                break;
            case TIM7_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
                break;
            case TIM12_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
                break;
            case TIM13_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
                break;
            case TIM14_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
                break;
            case WWDG_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;
                break;
            case SPI2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
                break;
            case SPI3_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
                break;
            case USART2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
                break;
            case USART3_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
                break;
            case UART4_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
                break;
            case UART5_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
                break;
            case I2C1_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
                break;
            case I2C2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
                break;
            case I2C3_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
                break;
            case CAN1_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
                break;
            case CAN2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;
                break;
            case PWR_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_PWREN;
                break;
            case DAC_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_DACEN;
                break;
#ifdef STM32F429xx
            case UART7_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_UART7EN;
                break;
            case UART8_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_UART8EN;
                break;
#endif
        }

        // APB2 peripherals
        {
            case TIM1_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
                break;
            case TIM8_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
                break;
            case USART1_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
                break;
            case USART6_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
                break;
            case ADC1_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
                break;
            case ADC2_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
                break;
            case ADC3_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
                break;
            case SDIO_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SDIOEN;
                break;
            case SPI1_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
                break;
#ifdef STM32F429xx
            case SPI4_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
                break;
#endif
            case SYSCFG_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
                break;
            case TIM9_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
                break;
            case TIM10_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
                break;
            case TIM11_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
                break;
#ifdef STM32F429xx
            case SPI5_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
                break;
            case SPI6_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SPI6EN;
                break;
            case SAI1_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;
                break;
            case LTDC_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_LTDCEN;
                break;
#endif
        }

        default:
            return false;
    }

    RCC_SYNC();

    return true;
}

inline bool ClockUtils::disablePeripheralClock(void* peripheral)
{
    miosix::FastInterruptDisableLock dLock;

    switch (reinterpret_cast<uint32_t>(peripheral))
    {
        // AHB1 peripherals
        {
            case GPIOA_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN;
                break;
            case GPIOB_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN;
                break;
            case GPIOC_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN;
                break;
            case GPIOD_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN;
                break;
            case GPIOE_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOEEN;
                break;
            case GPIOF_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOFEN;
                break;
            case GPIOG_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOGEN;
                break;
            case GPIOH_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOHEN;
                break;
            case GPIOI_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOIEN;
                break;
#ifdef STM32F429xx
            case GPIOJ_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOJEN;
                break;
            case GPIOK_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOKEN;
                break;
#endif
            case CRC_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;
                break;
            case BKPSRAM_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_BKPSRAMEN;
                break;
#ifndef _ARCH_CORTEXM3_STM32F2
            case CCMDATARAM_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_CCMDATARAMEN;
                break;
#endif
            case DMA1_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA1EN;
                break;
            case DMA2_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2EN;
                break;
#ifdef STM32F429xx
            case DMA2D_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2DEN;
                break;
#endif
#ifndef _ARCH_CORTEXM3_STM32F2
            case ETH_MAC_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_ETHMACEN;
                break;
#endif
            case USB_OTG_HS_PERIPH_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_OTGHSEN;
                break;
        }

        // AHB2 peripherals
        {
#ifndef _ARCH_CORTEXM3_STM32F2
            case DCMI_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_DCMIEN;
                break;
#endif
            case RNG_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_RNGEN;
                break;
            case USB_OTG_FS_PERIPH_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_OTGFSEN;
                break;
        }

        // APB1 peripherals
        {
            case TIM2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
                break;
            case TIM3_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN;
                break;
            case TIM4_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN;
                break;
            case TIM5_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN;
                break;
            case TIM6_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;
                break;
            case TIM7_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM7EN;
                break;
            case TIM12_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM12EN;
                break;
            case TIM13_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM13EN;
                break;
            case TIM14_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;
                break;
            case WWDG_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_WWDGEN;
                break;
            case SPI2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
                break;
            case SPI3_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
                break;
            case USART2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
                break;
            case USART3_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
                break;
            case UART4_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_UART4EN;
                break;
            case UART5_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_UART5EN;
                break;
            case I2C1_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
                break;
            case I2C2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
                break;
            case I2C3_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_I2C3EN;
                break;
            case CAN1_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_CAN1EN;
                break;
            case CAN2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_CAN2EN;
                break;
            case PWR_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;
                break;
            case DAC_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
                break;
#ifdef STM32F429xx
            case UART7_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_UART7EN;
                break;
            case UART8_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_UART8EN;
                break;
#endif
        }

        // APB2 peripherals
        {
            case TIM1_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
                break;
            case TIM8_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_TIM8EN;
                break;
            case USART1_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
                break;
            case USART6_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;
                break;
            case ADC1_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
                break;
            case ADC2_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_ADC2EN;
                break;
            case ADC3_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_ADC3EN;
                break;
            case SDIO_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SDIOEN;
                break;
            case SPI1_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
                break;
#ifdef STM32F429xx
            case SPI4_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SPI4EN;
                break;
#endif
            case SYSCFG_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN;
                break;
            case TIM9_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_TIM9EN;
                break;
            case TIM10_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN;
                break;
            case TIM11_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_TIM11EN;
                break;
#ifdef STM32F429xx
            case SPI5_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SPI5EN;
                break;
            case SPI6_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SPI6EN;
                break;
            case SAI1_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SAI1EN;
                break;
            case LTDC_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_LTDCEN;
                break;
#endif
        }

        default:
            return false;
    }

    RCC_SYNC();

    return true;
}

}  // namespace Boardcore
