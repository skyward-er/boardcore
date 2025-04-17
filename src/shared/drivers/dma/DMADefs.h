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
    /**
     * Here are defined the selectable streams.
     *
     * The problem is that some of these stream are used
     * by miosix. The corresponding IRQHandlers are already defined
     * in there, causing conflicts.
     * Moreover, the used streams differ from STM32F407xx to
     * STM32F767xx. That's why some streams are available only
     * for a particular board, or none (DMA2_Stream3 is not available
     * at all).
     */

    DMA1_Str0 = 0,

#ifndef STM32F407xx
    // This stream is used by miosix for STM32F407xx boards
    DMA1_Str1 = 1,
#endif  // STM32F407xx

    DMA1_Str2 = 2,

#ifndef STM32F407xx
    // This stream is used by miosix for STM32F407xx boards
    DMA1_Str3 = 3,
#endif  // STM32F407xx

    DMA1_Str4 = 4,
    DMA1_Str5 = 5,
    DMA1_Str6 = 6,
    DMA1_Str7 = 7,
    DMA2_Str0 = 8,
    DMA2_Str1 = 9,
    DMA2_Str2 = 10,
    // DMA2_Str3 = 11, // Always used by miosix
    DMA2_Str4 = 12,

#ifndef STM32F767xx
    // This stream is used by miosix for STM32F767xx boards
    DMA2_Str5 = 13,
#endif  // STM32F767xx

    DMA2_Str6 = 14,

#ifndef STM32F767xx
    // This stream is used by miosix for STM32F767xx boards
    DMA2_Str7 = 15,
#endif  // STM32F767xx
};

/**
 * @brief Channels selectable for each dma stream.
 */
enum class Channel : uint32_t
{
    // The first 8 channels are valid for all supported architectures
    CHANNEL0 = 0,
    CHANNEL1 = DMA_SxCR_CHSEL_0,
    CHANNEL2 = DMA_SxCR_CHSEL_1,
    CHANNEL3 = DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0,
    CHANNEL4 = DMA_SxCR_CHSEL_2,
    CHANNEL5 = DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_0,
    CHANNEL6 = DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1,
    CHANNEL7 = DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0,

// stm32f767 also has channels from 8 to 11
#ifdef STM32F767xx
    CHANNEL8  = DMA_SxCR_CHSEL_3,
    CHANNEL9  = DMA_SxCR_CHSEL_3 | DMA_SxCR_CHSEL_0,
    CHANNEL10 = DMA_SxCR_CHSEL_3 | DMA_SxCR_CHSEL_1,
    CHANNEL11 = DMA_SxCR_CHSEL_3 | DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0,
#endif  // STM32F767xx
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
    PE_QUADSPI,
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
    PE_I2C4_TX,
    PE_I2C4_RX,
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
    PE_SAI2_A,
    PE_SAI1_B,
    PE_SAI2_B,
    PE_DCMI,
    PE_SDIO,
    PE_SDMMC1,
    PE_SDMMC2,
    PE_CRYP_OUT,
    PE_CRYP_IN,
    PE_HASH_IN,
    PE_SPDIFRX_DT,
    PE_SPDIFRX_CS,
    PE_DFSDM1_FLT0,
    PE_DFSDM1_FLT1,
    PE_DFSDM1_FLT2,
    PE_DFSDM1_FLT3,
    PE_JPEG_IN,
    PE_JPEG_OUT,
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
 *
 * The actual initialization of mapPeripherals is board
 * specific, and can be found inside the "board_mappings"
 * folder.
 */
extern const std::multimap<Peripherals, std::pair<DMAStreamId, Channel>>
    mapPeripherals;

}  // namespace DMADefs

}  // namespace Boardcore
