/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Fabrizio Monti
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
#include <stdint.h>

#include <map>

namespace Boardcore
{

namespace DMADefs
{

enum class DMAStreamId : uint8_t
{
    // TODO: some streams are commented because the
    // corresponding irq handler is already defined
    // by miosix. For now those streams are not usable,
    // decide how to handle this situation.

    DMA1_Str0 = 0,
    // DMA1_Str1 = 1,
    DMA1_Str2 = 2,
    // DMA1_Str3 = 3,
    DMA1_Str4 = 4,
    DMA1_Str5 = 5,
    DMA1_Str6 = 6,
    DMA1_Str7 = 7,
    DMA2_Str0 = 8,
    DMA2_Str1 = 9,
    DMA2_Str2 = 10,
    // DMA2_Str3 = 11,
    DMA2_Str4 = 12,
    DMA2_Str5 = 13,
    DMA2_Str6 = 14,
    DMA2_Str7 = 15,
};

/**
 * @brief Mapping between `DMAStreamId` and the corresponding irq number.
 * This is needed because irq number values are not contiguous and they are
 * architecture dependent.
 */
const IRQn_Type irqNumberMapping[] = {
    DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn,
    DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn,
    DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn,
    DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn,
};

/**
 * @brief Channels selectable for each dma stream.
 */
enum class Channel : uint32_t
{
    CHANNEL0 = 0,
    CHANNEL1 = DMA_SxCR_CHSEL_0,
    CHANNEL2 = DMA_SxCR_CHSEL_1,
    CHANNEL3 = DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0,
    CHANNEL4 = DMA_SxCR_CHSEL_2,
    CHANNEL5 = DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_0,
    CHANNEL6 = DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1,
    CHANNEL7 = DMA_SxCR_CHSEL,
};

/**
 * @brief All the peripherals connected to dma.
 */
enum class Peripherals : uint8_t
{
    PE_MEM_ONLY,

    PE_SPI1_TX,
    PE_SPI1_RX,
    PE_SPI2_TX,
    PE_SPI2_RX,
    PE_SPI3_TX,
    PE_SPI3_RX,
    PE_SPI4_TX,
    PE_SPI4_RX,
    PE_SPI5_TX,
    PE_SPI5_RX,
    PE_SPI6_TX,
    PE_SPI6_RX,
    PE_USART1_TX,
    PE_USART1_RX,
    PE_USART2_TX,
    PE_USART2_RX,
    PE_USART3_TX,
    PE_USART3_RX,
    PE_UART4_TX,
    PE_UART4_RX,
    PE_UART5_TX,
    PE_UART5_RX,
    PE_USART6_TX,
    PE_USART6_RX,
    PE_UART7_TX,
    PE_UART7_RX,
    PE_UART8_TX,
    PE_UART8_RX,
    PE_I2C1_TX,
    PE_I2C1_RX,
    PE_I2C2_TX,
    PE_I2C2_RX,
    PE_I2C3_TX,
    PE_I2C3_RX,
    PE_I2S2_EXT_TX,
    PE_I2S2_EXT_RX,
    PE_I2S3_EXT_TX,
    PE_I2S3_EXT_RX,
    PE_TIM1_UP,
    PE_TIM1_TRIG,
    PE_TIM1_COM,
    PE_TIM1_CH1,
    PE_TIM1_CH2,
    PE_TIM1_CH3,
    PE_TIM1_CH4,
    PE_TIM2_UP,
    PE_TIM2_CH1,
    PE_TIM2_CH2,
    PE_TIM2_CH3,
    PE_TIM2_CH4,
    PE_TIM3_UP,
    PE_TIM3_TRIG,
    PE_TIM3_CH1,
    PE_TIM3_CH2,
    PE_TIM3_CH3,
    PE_TIM3_CH4,
    PE_TIM4_UP,
    PE_TIM4_CH1,
    PE_TIM4_CH2,
    PE_TIM4_CH3,
    PE_TIM5_UP,
    PE_TIM5_TRIG,
    PE_TIM5_CH1,
    PE_TIM5_CH2,
    PE_TIM5_CH3,
    PE_TIM5_CH4,
    PE_TIM6_UP,
    PE_TIM7_UP,
    PE_TIM8_UP,
    PE_TIM8_TRIG,
    PE_TIM8_COM,
    PE_TIM8_CH1,
    PE_TIM8_CH2,
    PE_TIM8_CH3,
    PE_TIM8_CH4,
    PE_DAC1,
    PE_DAC2,
    PE_ADC1,
    PE_ADC2,
    PE_ADC3,
    PE_SAI1_A,
    PE_SAI1_B,
    PE_DCMI,
    PE_SDIO,
    PE_CRYP_OUT,
    PE_CRYP_IN,
    PE_HASH_IN,

};

/**
 * @brief Maps the peripherals to the dma streams (and
 * the corresponding channel) that are connected with.
 */
const std::multimap<Peripherals, std::pair<DMAStreamId, Channel>>
    mapPeripherals = {

        // MEM-TO-MEM (only dma2 can perform mem-to-mem copy)
        {Peripherals::PE_MEM_ONLY, {DMAStreamId::DMA2_Str0, Channel::CHANNEL0}},
        {Peripherals::PE_MEM_ONLY, {DMAStreamId::DMA2_Str1, Channel::CHANNEL0}},
        {Peripherals::PE_MEM_ONLY, {DMAStreamId::DMA2_Str2, Channel::CHANNEL0}},
        // {Peripherals::PE_MEM_ONLY, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL0}}, // Stream currently not supported
        {Peripherals::PE_MEM_ONLY, {DMAStreamId::DMA2_Str4, Channel::CHANNEL0}},
        {Peripherals::PE_MEM_ONLY, {DMAStreamId::DMA2_Str5, Channel::CHANNEL0}},
        {Peripherals::PE_MEM_ONLY, {DMAStreamId::DMA2_Str6, Channel::CHANNEL0}},
        {Peripherals::PE_MEM_ONLY, {DMAStreamId::DMA2_Str7, Channel::CHANNEL0}},

        // SPI
        {Peripherals::PE_SPI1_TX, {DMAStreamId::DMA2_Str5, Channel::CHANNEL3}},
        // {Peripherals::PE_SPI1_TX, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL3}}, // Stream currently not supported
        {Peripherals::PE_SPI1_RX, {DMAStreamId::DMA2_Str2, Channel::CHANNEL3}},
        {Peripherals::PE_SPI1_RX, {DMAStreamId::DMA2_Str0, Channel::CHANNEL3}},

        {Peripherals::PE_SPI2_TX, {DMAStreamId::DMA1_Str4, Channel::CHANNEL0}},
        // {Peripherals::PE_SPI2_RX, {DMAStreamId::DMA1_Str3,
        // Channel::CHANNEL0}}, // Stream currently not supported

        {Peripherals::PE_SPI3_TX, {DMAStreamId::DMA1_Str5, Channel::CHANNEL0}},
        {Peripherals::PE_SPI3_TX, {DMAStreamId::DMA1_Str7, Channel::CHANNEL0}},
        {Peripherals::PE_SPI3_RX, {DMAStreamId::DMA1_Str0, Channel::CHANNEL0}},
        {Peripherals::PE_SPI3_RX, {DMAStreamId::DMA1_Str2, Channel::CHANNEL0}},

        // {Peripherals::PE_SPI4_TX, {DMAStreamId::DMA2_Str1,
        // Channel::CHANNEL4}}, // available on STM32F42xxx and STM32F43xxx only
        // {Peripherals::PE_SPI4_TX, {DMAStreamId::DMA2_Str4,
        // Channel::CHANNEL5}}, // available on STM32F42xxx and STM32F43xxx only
        // {Peripherals::PE_SPI4_RX, {DMAStreamId::DMA2_Str0,
        // Channel::CHANNEL4}}, // available on STM32F42xxx and STM32F43xxx only
        // {Peripherals::PE_SPI4_RX, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL5}}, // available on STM32F42xxx and STM32F43xxx only

        // {Peripherals::PE_SPI5_TX, {DMAStreamId::DMA2_Str4,
        // Channel::CHANNEL2}}, // available on STM32F42xxx and STM32F43xxx only
        // {Peripherals::PE_SPI5_TX, {DMAStreamId::DMA2_Str6,
        // Channel::CHANNEL7}}, // available on STM32F42xxx and STM32F43xxx only
        // {Peripherals::PE_SPI5_RX, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL2}}, // available on STM32F42xxx and STM32F43xxx only
        // {Peripherals::PE_SPI5_RX, {DMAStreamId::DMA2_Str5,
        // Channel::CHANNEL7}}, // available on STM32F42xxx and STM32F43xxx only

        // {Peripherals::PE_SPI6_TX, {DMAStreamId::DMA2_Str5,
        // Channel::CHANNEL1}}, // available on STM32F42xxx and STM32F43xxx only
        // {Peripherals::PE_SPI6_RX, {DMAStreamId::DMA2_Str6,
        // Channel::CHANNEL1}}, // available on STM32F42xxx and STM32F43xxx only

        // UART & USART
        {Peripherals::PE_USART1_TX,
         {DMAStreamId::DMA2_Str7, Channel::CHANNEL4}},
        {Peripherals::PE_USART1_RX,
         {DMAStreamId::DMA2_Str2, Channel::CHANNEL4}},
        {Peripherals::PE_USART1_RX,
         {DMAStreamId::DMA2_Str5, Channel::CHANNEL4}},

        {Peripherals::PE_USART2_TX,
         {DMAStreamId::DMA1_Str6, Channel::CHANNEL4}},
        {Peripherals::PE_USART2_RX,
         {DMAStreamId::DMA1_Str5, Channel::CHANNEL4}},

        // {Peripherals::PE_USART3_TX, {DMAStreamId::DMA1_Str3,
        // Channel::CHANNEL4}}, // Stream currently not supported
        {Peripherals::PE_USART3_TX,
         {DMAStreamId::DMA1_Str4, Channel::CHANNEL7}},
        // {Peripherals::PE_USART3_RX, {DMAStreamId::DMA1_Str1,
        // Channel::CHANNEL4}}, // Stream currently not supported

        {Peripherals::PE_UART4_TX, {DMAStreamId::DMA1_Str4, Channel::CHANNEL4}},
        {Peripherals::PE_UART4_RX, {DMAStreamId::DMA1_Str2, Channel::CHANNEL4}},

        {Peripherals::PE_UART5_TX, {DMAStreamId::DMA1_Str7, Channel::CHANNEL4}},
        {Peripherals::PE_UART5_RX, {DMAStreamId::DMA1_Str0, Channel::CHANNEL4}},

        // {Peripherals::PE_UART7_TX, {DMAStreamId::DMA1_Str1,
        // Channel::CHANNEL5}}, // available on STM32F42xxx and STM32F43xxx only
        // {Peripherals::PE_UART7_RX, {DMAStreamId::DMA1_Str3,
        // Channel::CHANNEL5}}, // available on STM32F42xxx and STM32F43xxx only

        // {Peripherals::PE_UART8_TX, {DMAStreamId::DMA1_Str0,
        // Channel::CHANNEL5}}, // available on STM32F42xxx and STM32F43xxx only
        // {Peripherals::PE_UART8_RX, {DMAStreamId::DMA1_Str6,
        // Channel::CHANNEL5}}, // available on STM32F42xxx and STM32F43xxx only

        {Peripherals::PE_USART6_TX,
         {DMAStreamId::DMA2_Str6, Channel::CHANNEL5}},
        {Peripherals::PE_USART6_TX,
         {DMAStreamId::DMA2_Str7, Channel::CHANNEL5}},
        {Peripherals::PE_USART6_RX,
         {DMAStreamId::DMA2_Str1, Channel::CHANNEL5}},
        {Peripherals::PE_USART6_RX,
         {DMAStreamId::DMA2_Str2, Channel::CHANNEL5}},

        // I2C
        {Peripherals::PE_I2C1_TX, {DMAStreamId::DMA1_Str6, Channel::CHANNEL1}},
        {Peripherals::PE_I2C1_TX, {DMAStreamId::DMA1_Str7, Channel::CHANNEL1}},
        {Peripherals::PE_I2C1_RX, {DMAStreamId::DMA1_Str0, Channel::CHANNEL1}},
        {Peripherals::PE_I2C1_RX, {DMAStreamId::DMA1_Str5, Channel::CHANNEL1}},

        {Peripherals::PE_I2C2_TX, {DMAStreamId::DMA1_Str7, Channel::CHANNEL7}},
        {Peripherals::PE_I2C2_RX, {DMAStreamId::DMA1_Str2, Channel::CHANNEL7}},
        // {Peripherals::PE_I2C2_RX, {DMAStreamId::DMA1_Str3,
        // Channel::CHANNEL7}}, // Stream currently not supported

        {Peripherals::PE_I2C3_TX, {DMAStreamId::DMA1_Str4, Channel::CHANNEL3}},
        {Peripherals::PE_I2C3_RX, {DMAStreamId::DMA1_Str2, Channel::CHANNEL3}},

        {Peripherals::PE_I2S2_EXT_TX,
         {DMAStreamId::DMA1_Str4, Channel::CHANNEL2}},
        // {Peripherals::PE_I2S2_EXT_RX, {DMAStreamId::DMA1_Str3,
        // Channel::CHANNEL3}}, // Stream currently not supported

        {Peripherals::PE_I2S3_EXT_TX,
         {DMAStreamId::DMA1_Str5, Channel::CHANNEL2}},
        {Peripherals::PE_I2S3_EXT_RX,
         {DMAStreamId::DMA1_Str2, Channel::CHANNEL2}},
        {Peripherals::PE_I2S3_EXT_RX,
         {DMAStreamId::DMA1_Str0, Channel::CHANNEL3}},

        // TIMERS
        {Peripherals::PE_TIM1_UP, {DMAStreamId::DMA2_Str5, Channel::CHANNEL6}},
        {Peripherals::PE_TIM1_TRIG,
         {DMAStreamId::DMA2_Str0, Channel::CHANNEL6}},
        {Peripherals::PE_TIM1_TRIG,
         {DMAStreamId::DMA2_Str4, Channel::CHANNEL6}},
        {Peripherals::PE_TIM1_COM, {DMAStreamId::DMA2_Str4, Channel::CHANNEL6}},
        {Peripherals::PE_TIM1_CH1, {DMAStreamId::DMA2_Str6, Channel::CHANNEL0}},
        {Peripherals::PE_TIM1_CH1, {DMAStreamId::DMA2_Str1, Channel::CHANNEL6}},
        // {Peripherals::PE_TIM1_CH1, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL6}}, // Stream currently not supported
        {Peripherals::PE_TIM1_CH2, {DMAStreamId::DMA2_Str6, Channel::CHANNEL0}},
        {Peripherals::PE_TIM1_CH2, {DMAStreamId::DMA2_Str2, Channel::CHANNEL6}},
        {Peripherals::PE_TIM1_CH3, {DMAStreamId::DMA2_Str6, Channel::CHANNEL0}},
        {Peripherals::PE_TIM1_CH3, {DMAStreamId::DMA2_Str6, Channel::CHANNEL6}},
        {Peripherals::PE_TIM1_CH4, {DMAStreamId::DMA2_Str4, Channel::CHANNEL6}},

        // {Peripherals::PE_TIM2_UP, {DMAStreamId::DMA1_Str1,
        // Channel::CHANNEL3}}, // Stream currently not supported
        {Peripherals::PE_TIM2_UP, {DMAStreamId::DMA1_Str7, Channel::CHANNEL3}},
        {Peripherals::PE_TIM2_CH1, {DMAStreamId::DMA1_Str5, Channel::CHANNEL3}},
        {Peripherals::PE_TIM2_CH2, {DMAStreamId::DMA1_Str6, Channel::CHANNEL3}},
        // {Peripherals::PE_TIM2_CH3, {DMAStreamId::DMA1_Str1,
        // Channel::CHANNEL3}}, // Stream currently not supported
        {Peripherals::PE_TIM2_CH4, {DMAStreamId::DMA1_Str6, Channel::CHANNEL3}},
        {Peripherals::PE_TIM2_CH4, {DMAStreamId::DMA1_Str7, Channel::CHANNEL3}},

        {Peripherals::PE_TIM3_UP, {DMAStreamId::DMA1_Str2, Channel::CHANNEL5}},
        {Peripherals::PE_TIM3_TRIG,
         {DMAStreamId::DMA1_Str4, Channel::CHANNEL5}},
        {Peripherals::PE_TIM3_CH1, {DMAStreamId::DMA1_Str4, Channel::CHANNEL5}},
        {Peripherals::PE_TIM3_CH2, {DMAStreamId::DMA1_Str5, Channel::CHANNEL5}},
        {Peripherals::PE_TIM3_CH3, {DMAStreamId::DMA1_Str7, Channel::CHANNEL5}},
        {Peripherals::PE_TIM3_CH4, {DMAStreamId::DMA1_Str2, Channel::CHANNEL5}},

        {Peripherals::PE_TIM4_UP, {DMAStreamId::DMA1_Str6, Channel::CHANNEL2}},
        {Peripherals::PE_TIM4_CH1, {DMAStreamId::DMA1_Str0, Channel::CHANNEL2}},
        // {Peripherals::PE_TIM4_CH2, {DMAStreamId::DMA1_Str3,
        // Channel::CHANNEL2}}, // Stream currently not supported
        {Peripherals::PE_TIM4_CH3, {DMAStreamId::DMA1_Str7, Channel::CHANNEL2}},

        {Peripherals::PE_TIM5_UP, {DMAStreamId::DMA1_Str0, Channel::CHANNEL6}},
        {Peripherals::PE_TIM5_UP, {DMAStreamId::DMA1_Str6, Channel::CHANNEL6}},
        // {Peripherals::PE_TIM5_TRIG, {DMAStreamId::DMA1_Str1,
        // Channel::CHANNEL6}}, // Stream currently not supported
        // {Peripherals::PE_TIM5_TRIG, {DMAStreamId::DMA1_Str3,
        // Channel::CHANNEL6}}, // Stream currently not supported
        {Peripherals::PE_TIM5_CH1, {DMAStreamId::DMA1_Str2, Channel::CHANNEL6}},
        {Peripherals::PE_TIM5_CH2, {DMAStreamId::DMA1_Str4, Channel::CHANNEL6}},
        {Peripherals::PE_TIM5_CH3, {DMAStreamId::DMA1_Str0, Channel::CHANNEL6}},
        // {Peripherals::PE_TIM5_CH4, {DMAStreamId::DMA1_Str1,
        // Channel::CHANNEL6}}, // Stream currently not supported
        // {Peripherals::PE_TIM5_CH4, {DMAStreamId::DMA1_Str3,
        // Channel::CHANNEL6}}, // Stream currently not supported

        // {Peripherals::PE_TIM6_UP, {DMAStreamId::DMA1_Str1,
        // Channel::CHANNEL7}}, // Stream currently not supported

        {Peripherals::PE_TIM7_UP, {DMAStreamId::DMA1_Str2, Channel::CHANNEL1}},
        {Peripherals::PE_TIM7_UP, {DMAStreamId::DMA1_Str4, Channel::CHANNEL1}},

        {Peripherals::PE_TIM8_UP, {DMAStreamId::DMA2_Str1, Channel::CHANNEL7}},
        {Peripherals::PE_TIM8_TRIG,
         {DMAStreamId::DMA2_Str7, Channel::CHANNEL7}},
        {Peripherals::PE_TIM8_COM, {DMAStreamId::DMA2_Str7, Channel::CHANNEL7}},
        {Peripherals::PE_TIM8_CH1, {DMAStreamId::DMA2_Str2, Channel::CHANNEL0}},
        {Peripherals::PE_TIM8_CH1, {DMAStreamId::DMA2_Str2, Channel::CHANNEL7}},
        {Peripherals::PE_TIM8_CH2, {DMAStreamId::DMA2_Str2, Channel::CHANNEL0}},
        // {Peripherals::PE_TIM8_CH2, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL7}}, // Stream currently not supported
        {Peripherals::PE_TIM8_CH3, {DMAStreamId::DMA2_Str2, Channel::CHANNEL0}},
        {Peripherals::PE_TIM8_CH3, {DMAStreamId::DMA2_Str4, Channel::CHANNEL7}},
        {Peripherals::PE_TIM8_CH4, {DMAStreamId::DMA2_Str7, Channel::CHANNEL7}},

        // Others
        {Peripherals::PE_DAC1, {DMAStreamId::DMA1_Str5, Channel::CHANNEL7}},
        {Peripherals::PE_DAC2, {DMAStreamId::DMA1_Str6, Channel::CHANNEL7}},

        {Peripherals::PE_ADC1, {DMAStreamId::DMA2_Str0, Channel::CHANNEL0}},
        {Peripherals::PE_ADC1, {DMAStreamId::DMA2_Str4, Channel::CHANNEL0}},

        {Peripherals::PE_ADC2, {DMAStreamId::DMA2_Str2, Channel::CHANNEL1}},
        // {Peripherals::PE_ADC2, {DMAStreamId::DMA2_Str3, Channel::CHANNEL1}},
        // // Stream currently not supported

        {Peripherals::PE_ADC3, {DMAStreamId::DMA2_Str0, Channel::CHANNEL2}},
        {Peripherals::PE_ADC3, {DMAStreamId::DMA2_Str1, Channel::CHANNEL2}},

        // {Peripherals::PE_SAI1_A, {DMAStreamId::DMA2_Str1,
        // Channel::CHANNEL0}}, // available on STM32F42xxx and STM32F43xxx only
        // {Peripherals::PE_SAI1_A, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL0}}, // available on STM32F42xxx and STM32F43xxx only

        // {Peripherals::PE_SAI1_B, {DMAStreamId::DMA2_Str5,
        // Channel::CHANNEL0}}, // available on STM32F42xxx and STM32F43xxx only
        // {Peripherals::PE_SAI1_B, {DMAStreamId::DMA2_Str4,
        // Channel::CHANNEL1}}, // available on STM32F42xxx and STM32F43xxx only

        {Peripherals::PE_DCMI, {DMAStreamId::DMA2_Str1, Channel::CHANNEL1}},
        {Peripherals::PE_DCMI, {DMAStreamId::DMA2_Str7, Channel::CHANNEL1}},

        // {Peripherals::PE_SDIO, {DMAStreamId::DMA2_Str3, Channel::CHANNEL4}},
        // // Stream currently not supported
        {Peripherals::PE_SDIO, {DMAStreamId::DMA2_Str6, Channel::CHANNEL4}},

        {Peripherals::PE_CRYP_OUT, {DMAStreamId::DMA2_Str5, Channel::CHANNEL2}},
        {Peripherals::PE_CRYP_IN, {DMAStreamId::DMA2_Str6, Channel::CHANNEL2}},

        {Peripherals::PE_HASH_IN, {DMAStreamId::DMA2_Str7, Channel::CHANNEL2}},
};

}  // namespace DMADefs

}  // namespace Boardcore