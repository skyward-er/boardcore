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
#if _ARCH_CORTEXM3_STM32F1 | _ARCH_CORTEXM4_STM32L4
        const uint32_t ppre1 = 8;
#elif _ARCH_CORTEXM3_STM32F2 | _ARCH_CORTEXM4_STM32F4 | _ARCH_CORTEXM7_STM32F7
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
#if _ARCH_CORTEXM3_STM32F1 | _ARCH_CORTEXM4_STM32L4
        const uint32_t ppre2 = 11;
#elif _ARCH_CORTEXM3_STM32F2 | _ARCH_CORTEXM4_STM32F4 | _ARCH_CORTEXM7_STM32F7
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
#if _ARCH_CORTEXM3_STM32F1 | _ARCH_CORTEXM4_STM32L4
        const uint32_t ppre1 = 8;
#elif _ARCH_CORTEXM3_STM32F2 | _ARCH_CORTEXM4_STM32F4 | _ARCH_CORTEXM7_STM32F7
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
#if _ARCH_CORTEXM3_STM32F1 | _ARCH_CORTEXM4_STM32L4
        const uint32_t ppre2 = 11;
#elif _ARCH_CORTEXM3_STM32F2 | _ARCH_CORTEXM4_STM32F4 | _ARCH_CORTEXM7_STM32F7
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
#ifndef _ARCH_CORTEXM4_STM32L4
#ifdef GPIOA_BASE
            case GPIOA_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
                break;
#endif
#ifdef GPIOB_BASE
            case GPIOB_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
                break;
#endif
#ifdef GPIOC_BASE
            case GPIOC_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
                break;
#endif
#ifdef GPIOD_BASE
            case GPIOD_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
                break;
#endif
#ifdef GPIOE_BASE
            case GPIOE_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
                break;
#endif
#ifdef GPIOF_BASE
            case GPIOF_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
                break;
#endif
#ifdef GPIOG_BASE
            case GPIOG_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
                break;
#endif
#ifdef GPIOH_BASE
            case GPIOH_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
                break;
#endif
#ifdef GPIOI_BASE
            case GPIOI_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
                break;
#endif
#ifdef GPIOJ_BASE
            case GPIOJ_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOJEN;
                break;
#endif
#ifdef GPIOK_BASE
            case GPIOK_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOKEN;
                break;
#endif
#else
#ifdef GPIOA_BASE
            case GPIOA_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
                break;
#endif
#ifdef GPIOB_BASE
            case GPIOB_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
                break;
#endif
#ifdef GPIOC_BASE
            case GPIOC_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
                break;
#endif
#ifdef GPIOD_BASE
            case GPIOD_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
                break;
#endif
#ifdef GPIOE_BASE
            case GPIOE_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
                break;
#endif
#ifdef GPIOF_BASE
            case GPIOF_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
                break;
#endif
#ifdef GPIOG_BASE
            case GPIOG_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;
                break;
#endif
#ifdef GPIOH_BASE
            case GPIOH_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_GPIOHEN;
                break;
#endif
#ifdef GPIOI_BASE
            case GPIOI_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_GPIOIEN;
                break;
#endif
#ifdef GPIOJ_BASE
            case GPIOJ_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_GPIOJEN;
                break;
#endif
#ifdef GPIOK_BASE
            case GPIOK_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_GPIOKEN;
                break;
#endif
#endif
#ifdef CRC_BASE
            case CRC_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
                break;
#endif
#ifdef BKPSRAM_BASE
            case BKPSRAM_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
                break;
#endif
// In the CMSIS version used in the kernel, the CCMDATARAM_BASE macro is defined
// for some microcontrollers that do not have the Core Coupled Memory while
// the RCC_AHB1ENR_CCMDATARAMEN is correctly not defined in such cases. To fix
// the error, instead of checking for CCMDATARAM_BASE with #ifdef like for the
// other case statements, I check directly for RCC_AHB1ENR_CCMDATARAMEN.
#ifdef RCC_AHB1ENR_CCMDATARAMEN
            case CCMDATARAM_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_CCMDATARAMEN;
                break;
#endif
#ifdef DMA1_BASE
            case DMA1_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
                break;
#endif
#ifdef DMA2_BASE
            case DMA2_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
                break;
#endif
#ifdef DMA2D_BASE
            case DMA2D_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_DMA2DEN;
                break;
#endif
#ifdef ETH_MAC_BASE
            case ETH_MAC_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACEN;
                break;
#endif
#ifdef USB_OTG_HS_PERIPH_BASE
            case USB_OTG_HS_PERIPH_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_OTGHSEN;
                break;
#endif
        }

        // AHB2 peripherals
        {
#ifdef DCMI_BASE
            case DCMI_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN;
                break;
#endif
#ifdef RNG_BASE
            case RNG_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
                break;
#endif
#ifdef USB_OTG_FS_PERIPH_BASE
            case USB_OTG_FS_PERIPH_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
                break;
#endif
        }

        // AHB3 peripherals
        {
#ifdef QSPI_BASE
            case QSPI_BASE:
                RCC->AHB3ENR |= RCC_AHB3ENR_QSPIEN;
                break;
#endif
        }

        // APB1 peripherals
        {
#ifndef _ARCH_CORTEXM4_STM32L4
#ifdef TIM2_BASE
            case TIM2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
                break;
#endif
#ifdef TIM3_BASE
            case TIM3_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
                break;
#endif
#ifdef TIM4_BASE
            case TIM4_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
                break;
#endif
#ifdef TIM5_BASE
            case TIM5_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
                break;
#endif
#ifdef TIM6_BASE
            case TIM6_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
                break;
#endif
#ifdef TIM7_BASE
            case TIM7_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
                break;
#endif
#ifdef TIM12_BASE
            case TIM12_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
                break;
#endif
#ifdef TIM13_BASE
            case TIM13_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
                break;
#endif
#ifdef TIM14_BASE
            case TIM14_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
                break;
#endif
#else
#ifdef TIM2_BASE
            case TIM2_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
                break;
#endif
#ifdef TIM3_BASE
            case TIM3_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
                break;
#endif
#ifdef TIM4_BASE
            case TIM4_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
                break;
#endif
#ifdef TIM5_BASE
            case TIM5_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
                break;
#endif
#ifdef TIM6_BASE
            case TIM6_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
                break;
#endif
#ifdef TIM7_BASE
            case TIM7_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
                break;
#endif
#ifdef TIM12_BASE
            case TIM12_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM12EN;
                break;
#endif
#ifdef TIM13_BASE
            case TIM13_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM13EN;
                break;
#endif
#ifdef TIM14_BASE
            case TIM14_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_TIM14EN;
                break;
#endif
#endif
// RTC register interface gate only on stm32f7 micro controllers
#if defined(RTC_BASE) && defined(_ARCH_CORTEXM7_STM32F7)
            case RTC_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_RTCEN;
                break;
#endif
#ifndef _ARCH_CORTEXM4_STM32L4
#ifdef WWDG_BASE
            case WWDG_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;
                break;
#endif
#ifdef SPI2_BASE
            case SPI2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
                break;
#endif
#ifdef SPI3_BASE
            case SPI3_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
                break;
#endif
#ifdef USART2_BASE
            case USART2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
                break;
#endif
#ifdef USART3_BASE
            case USART3_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
                break;
#endif
#ifdef UART4_BASE
            case UART4_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
                break;
#endif
#ifdef UART5_BASE
            case UART5_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
                break;
#endif
#ifdef I2C1_BASE
            case I2C1_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
                break;
#endif
#ifdef I2C2_BASE
            case I2C2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
                break;
#endif
#ifdef I2C3_BASE
            case I2C3_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
                break;
#endif
#ifdef CAN1_BASE
            case CAN1_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
                break;
#endif
#ifdef CAN2_BASE
            case CAN2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;
                break;
#endif
#ifdef PWR_BASE
            case PWR_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_PWREN;
                break;
#endif
#ifdef DAC_BASE
            case DAC_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_DACEN;
                break;
#endif
#ifdef UART7_BASE
            case UART7_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_UART7EN;
                break;
#endif
#ifdef UART8_BASE
            case UART8_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_UART8EN;
                break;
#endif
#else
#ifdef WWDG_BASE
            case WWDG_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_WWDGEN;
                break;
#endif
#ifdef SPI2_BASE
            case SPI2_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
                break;
#endif
#ifdef SPI3_BASE
            case SPI3_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;
                break;
#endif
#ifdef USART2_BASE
            case USART2_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
                break;
#endif
#ifdef USART3_BASE
            case USART3_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
                break;
#endif
#ifdef UART4_BASE
            case UART4_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;
                break;
#endif
#ifdef UART5_BASE
            case UART5_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_UART5EN;
                break;
#endif
#ifdef I2C1_BASE
            case I2C1_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
                break;
#endif
#ifdef I2C2_BASE
            case I2C2_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
                break;
#endif
#ifdef I2C3_BASE
            case I2C3_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_I2C3EN;
                break;
#endif
#ifdef CAN1_BASE
            case CAN1_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_CAN1EN;
                break;
#endif
#ifdef CAN2_BASE
            case CAN2_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_CAN2EN;
                break;
#endif
#ifdef PWR_BASE
            case PWR_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
                break;
#endif
#ifdef DAC_BASE
            case DAC_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;
                break;
#endif
#ifdef UART7_BASE
            case UART7_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_UART7EN;
                break;
#endif
#ifdef UART8_BASE
            case UART8_BASE:
                RCC->APB1ENR1 |= RCC_APB1ENR1_UART8EN;
                break;
#endif
#endif
        }

        // APB2 peripherals
        {
#ifdef TIM1_BASE
            case TIM1_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
                break;
#endif
#ifdef TIM8_BASE
            case TIM8_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
                break;
#endif
#ifdef USART1_BASE
            case USART1_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
                break;
#endif
#ifdef USART6_BASE
            case USART6_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
                break;
#endif
#ifndef _ARCH_CORTEXM4_STM32L4
#ifdef ADC1_BASE
            case ADC1_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
                break;
#endif
#endif
#ifdef ADC2_BASE
            case ADC2_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
                break;
#endif
#ifdef ADC3_BASE
            case ADC3_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
                break;
#endif
#ifdef SDIO_BASE
            case SDIO_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SDIOEN;
                break;
#endif
#ifdef SPI1_BASE
            case SPI1_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
                break;
#endif
#ifdef SPI4_BASE
            case SPI4_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
                break;
#endif
#ifdef SYSCFG_BASE
            case SYSCFG_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
                break;
#endif
#ifdef TIM9_BASE
            case TIM9_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
                break;
#endif
#ifdef TIM10_BASE
            case TIM10_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
                break;
#endif
#ifdef TIM11_BASE
            case TIM11_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
                break;
#endif
#ifdef SPI5_BASE
            case SPI5_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
                break;
#endif
#ifdef SPI6_BASE
            case SPI6_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SPI6EN;
                break;
#endif
#ifdef SAI1_BASE
            case SAI1_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;
                break;
#endif
#ifdef LTDC_BASE
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
#ifndef _ARCH_CORTEXM4_STM32L4
#ifdef GPIOA_BASE
            case GPIOA_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN;
                break;
#endif
#ifdef GPIOB_BASE
            case GPIOB_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN;
                break;
#endif
#ifdef GPIOC_BASE
            case GPIOC_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN;
                break;
#endif
#ifdef GPIOD_BASE
            case GPIOD_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN;
                break;
#endif
#ifdef GPIOE_BASE
            case GPIOE_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOEEN;
                break;
#endif
#ifdef GPIOF_BASE
            case GPIOF_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOFEN;
                break;
#endif
#ifdef GPIOG_BASE
            case GPIOG_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOGEN;
                break;
#endif
#ifdef GPIOH_BASE
            case GPIOH_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOHEN;
                break;
#endif
#ifdef GPIOI_BASE
            case GPIOI_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOIEN;
                break;
#endif
#ifdef GPIOJ_BASE
            case GPIOJ_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOJEN;
                break;
#endif
#ifdef GPIOK_BASE
            case GPIOK_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOKEN;
                break;
#endif
#else
#ifdef GPIOA_BASE
            case GPIOA_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOAEN;
                break;
#endif
#ifdef GPIOB_BASE
            case GPIOB_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOBEN;
                break;
#endif
#ifdef GPIOC_BASE
            case GPIOC_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOCEN;
                break;
#endif
#ifdef GPIOD_BASE
            case GPIOD_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIODEN;
                break;
#endif
#ifdef GPIOE_BASE
            case GPIOE_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOEEN;
                break;
#endif
#ifdef GPIOF_BASE
            case GPIOF_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOFEN;
                break;
#endif
#ifdef GPIOG_BASE
            case GPIOG_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOGEN;
                break;
#endif
#ifdef GPIOH_BASE
            case GPIOH_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOHEN;
                break;
#endif
#ifdef GPIOI_BASE
            case GPIOI_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOIEN;
                break;
#endif
#ifdef GPIOJ_BASE
            case GPIOJ_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOJEN;
                break;
#endif
#ifdef GPIOK_BASE
            case GPIOK_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOKEN;
                break;
#endif
#endif
#ifdef CRC_BASE
            case CRC_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;
                break;
#endif
#ifdef BKPSRAM_BASE
            case BKPSRAM_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_BKPSRAMEN;
                break;
#endif
// In the CMSIS version used in the kernel, the CCMDATARAM_BASE macro is defined
// for some microcontrollers that do not have the Core Coupled Memory while
// the RCC_AHB1ENR_CCMDATARAMEN is correctly not defined in such cases. To fix
// the error, instead of checking for CCMDATARAM_BASE with #ifdef like for the
// other case statements, I check directly for RCC_AHB1ENR_CCMDATARAMEN.
#ifdef RCC_AHB1ENR_CCMDATARAMEN
            case CCMDATARAM_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_CCMDATARAMEN;
                break;
#endif
#ifdef DMA1_BASE
            case DMA1_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA1EN;
                break;
#endif
#ifdef DMA2_BASE
            case DMA2_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2EN;
                break;
#endif
#ifdef DMA2D_BASE
            case DMA2D_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2DEN;
                break;
#endif
#ifdef ETH_MAC_BASE
            case ETH_MAC_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_ETHMACEN;
                break;
#endif
#ifdef USB_OTG_HS_PERIPH_BASE
            case USB_OTG_HS_PERIPH_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_OTGHSEN;
                break;
#endif
        }

        // AHB2 peripherals
        {
#ifdef DCMI_BASE
            case DCMI_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_DCMIEN;
                break;
#endif
#ifdef RNG_BASE
            case RNG_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_RNGEN;
                break;
#endif
#ifdef USB_OTG_FS_PERIPH_BASE
            case USB_OTG_FS_PERIPH_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_OTGFSEN;
                break;
#endif
        }

        // AHB3 peripherals
        {
#ifdef QSPI_BASE
            case QSPI_BASE:
                RCC->AHB3ENR &= ~RCC_AHB3ENR_QSPIEN;
                break;
#endif
        }

        // APB1 peripherals
        {
#ifndef _ARCH_CORTEXM4_STM32L4
#ifdef TIM2_BASE
            case TIM2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
                break;
#endif
#ifdef TIM3_BASE
            case TIM3_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN;
                break;
#endif
#ifdef TIM4_BASE
            case TIM4_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN;
                break;
#endif
#ifdef TIM5_BASE
            case TIM5_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN;
                break;
#endif
#ifdef TIM6_BASE
            case TIM6_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;
                break;
#endif
#ifdef TIM7_BASE
            case TIM7_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM7EN;
                break;
#endif
#ifdef TIM12_BASE
            case TIM12_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM12EN;
                break;
#endif
#ifdef TIM13_BASE
            case TIM13_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM13EN;
                break;
#endif
#ifdef TIM14_BASE
            case TIM14_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;
                break;
#endif
#else
#ifdef TIM2_BASE
            case TIM2_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM2EN;
                break;
#endif
#ifdef TIM3_BASE
            case TIM3_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM3EN;
                break;
#endif
#ifdef TIM4_BASE
            case TIM4_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM4EN;
                break;
#endif
#ifdef TIM5_BASE
            case TIM5_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM5EN;
                break;
#endif
#ifdef TIM6_BASE
            case TIM6_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM6EN;
                break;
#endif
#ifdef TIM7_BASE
            case TIM7_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM7EN;
                break;
#endif
#ifdef TIM12_BASE
            case TIM12_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM12EN;
                break;
#endif
#ifdef TIM13_BASE
            case TIM13_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM13EN;
                break;
#endif
#ifdef TIM14_BASE
            case TIM14_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM14EN;
                break;
#endif
#endif
// RTC register interface gate only on stm32f7 micro controllers
#if defined(RTC_BASE) && defined(_ARCH_CORTEXM7_STM32F7)
            case RTC_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_RTCEN;
                break;
#endif
#ifndef _ARCH_CORTEXM4_STM32L4
#ifdef WWDG_BASE
            case WWDG_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_WWDGEN;
                break;
#endif
#ifdef SPI2_BASE
            case SPI2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
                break;
#endif
#ifdef SPI3_BASE
            case SPI3_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
                break;
#endif
#ifdef USART2_BASE
            case USART2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
                break;
#endif
#ifdef USART3_BASE
            case USART3_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
                break;
#endif
#ifdef UART4_BASE
            case UART4_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_UART4EN;
                break;
#endif
#ifdef UART5_BASE
            case UART5_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_UART5EN;
                break;
#endif
#ifdef I2C1_BASE
            case I2C1_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
                break;
#endif
#ifdef I2C2_BASE
            case I2C2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
                break;
#endif
#ifdef I2C3_BASE
            case I2C3_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_I2C3EN;
                break;
#endif
#ifdef CAN1_BASE
            case CAN1_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_CAN1EN;
                break;
#endif
#ifdef CAN2_BASE
            case CAN2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_CAN2EN;
                break;
#endif
#ifdef PWR_BASE
            case PWR_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;
                break;
#endif
#ifdef DAC_BASE
            case DAC_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
                break;
#endif
#ifdef UART7_BASE
            case UART7_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_UART7EN;
                break;
#endif
#ifdef UART8_BASE
            case UART8_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_UART8EN;
                break;
#endif
#else
#ifdef WWDG_BASE
            case WWDG_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_WWDGEN;
                break;
#endif
#ifdef SPI2_BASE
            case SPI2_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_SPI2EN;
                break;
#endif
#ifdef SPI3_BASE
            case SPI3_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_SPI3EN;
                break;
#endif
#ifdef USART2_BASE
            case USART2_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART2EN;
                break;
#endif
#ifdef USART3_BASE
            case USART3_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART3EN;
                break;
#endif
#ifdef UART4_BASE
            case UART4_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_UART4EN;
                break;
#endif
#ifdef UART5_BASE
            case UART5_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_UART5EN;
                break;
#endif
#ifdef I2C1_BASE
            case I2C1_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C1EN;
                break;
#endif
#ifdef I2C2_BASE
            case I2C2_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C2EN;
                break;
#endif
#ifdef I2C3_BASE
            case I2C3_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C3EN;
                break;
#endif
#ifdef CAN1_BASE
            case CAN1_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_CAN1EN;
                break;
#endif
#ifdef CAN2_BASE
            case CAN2_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_CAN2EN;
                break;
#endif
#ifdef PWR_BASE
            case PWR_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_PWREN;
                break;
#endif
#ifdef DAC_BASE
            case DAC_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_DAC1EN;
                break;
#endif
#ifdef UART7_BASE
            case UART7_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_UART7EN;
                break;
#endif
#ifdef UART8_BASE
            case UART8_BASE:
                RCC->APB1ENR1 &= ~RCC_APB1ENR1_UART8EN;
                break;
#endif
#endif
        }

        // APB2 peripherals
        {
#ifdef TIM1_BASE
            case TIM1_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
                break;
#endif
#ifdef TIM8_BASE
            case TIM8_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_TIM8EN;
                break;
#endif
#ifdef USART1_BASE
            case USART1_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
                break;
#endif
#ifdef USART6_BASE
            case USART6_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;
                break;
#endif
#ifndef _ARCH_CORTEXM4_STM32L4
#ifdef ADC1_BASE
            case ADC1_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
                break;
#endif
#endif
#ifdef ADC2_BASE
            case ADC2_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_ADC2EN;
                break;
#endif
#ifdef ADC3_BASE
            case ADC3_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_ADC3EN;
                break;
#endif
#ifdef SDIO_BASE
            case SDIO_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SDIOEN;
                break;
#endif
#ifdef SPI1_BASE
            case SPI1_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
                break;
#endif
#ifdef SPI4_BASE
            case SPI4_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SPI4EN;
                break;
#endif
#ifdef SYSCFG_BASE
            case SYSCFG_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN;
                break;
#endif
#ifdef TIM9_BASE
            case TIM9_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_TIM9EN;
                break;
#endif
#ifdef TIM10_BASE
            case TIM10_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN;
                break;
#endif
#ifdef TIM11_BASE
            case TIM11_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_TIM11EN;
                break;
#endif
#ifdef SPI5_BASE
            case SPI5_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SPI5EN;
                break;
#endif
#ifdef SPI6_BASE
            case SPI6_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SPI6EN;
                break;
#endif
#ifdef SAI1_BASE
            case SAI1_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SAI1EN;
                break;
#endif
#ifdef LTDC_BASE
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
