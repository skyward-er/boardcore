/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Alberto Nidasio, Fabrizio Monti
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

// For some reason these defines are missing
// in the CMSIS STM32F4xx file
#ifdef STM32F407xx

#ifndef DMA_SxCR_MSIZE_Pos
#define DMA_SxCR_MSIZE_Pos (13U)
#endif

#ifndef DMA_SxCR_PSIZE_Pos
#define DMA_SxCR_PSIZE_Pos (11U)
#endif

#endif  // STM32F407xx

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
 * @brief Mapping between `DMAStreamId` and the corresponding irq number.
 * This is needed because irq number values are not contiguous and they are
 * architecture dependent.
 */
extern const IRQn_Type irqNumberMapping[];

/**
 * @brief Maps the peripherals to the dma streams (and
 * the corresponding channel) that are connected with.
 */
extern const std::multimap<Peripherals, std::pair<DMAStreamId, Channel>>
    mapPeripherals;

}  // namespace DMADefs

}  // namespace Boardcore
