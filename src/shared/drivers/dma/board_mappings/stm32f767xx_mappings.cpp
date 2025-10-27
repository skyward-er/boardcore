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

#include "../DMADefs.h"

namespace Boardcore
{

namespace DMADefs
{
const std::multimap<Peripherals, std::pair<DMAStreamId, Channel>>
    mapPeripherals = {

        /**
         * Here are defined the mappings between peripherals and
         * streams.
         *
         * The problem is that some of these stream are used
         * by miosix. The corresponding IRQHandlers are already defined
         * in there, causing conflicts.
         * For this reason the unavailable mappings are commented out.
         */

        // MEM-TO-MEM (only dma2 can perform mem-to-mem copy)
        {Peripherals::PE_MEM_ONLY, {DMAStreamId::DMA2_Str0, Channel::CHANNEL0}},
        {Peripherals::PE_MEM_ONLY, {DMAStreamId::DMA2_Str1, Channel::CHANNEL0}},
        {Peripherals::PE_MEM_ONLY, {DMAStreamId::DMA2_Str2, Channel::CHANNEL0}},
        // {Peripherals::PE_MEM_ONLY, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL0}},
        {Peripherals::PE_MEM_ONLY, {DMAStreamId::DMA2_Str4, Channel::CHANNEL0}},
        // {Peripherals::PE_MEM_ONLY, {DMAStreamId::DMA2_Str5,
        // Channel::CHANNEL0}},
        {Peripherals::PE_MEM_ONLY, {DMAStreamId::DMA2_Str6, Channel::CHANNEL0}},
        // {Peripherals::PE_MEM_ONLY, {DMAStreamId::DMA2_Str7,
        // Channel::CHANNEL0}},

        // SPI
        // {Peripherals::PE_SPI1_TX, {DMAStreamId::DMA2_Str5,
        // Channel::CHANNEL3}},
        // {Peripherals::PE_SPI1_TX, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL3}},
        {Peripherals::PE_SPI1_RX, {DMAStreamId::DMA2_Str2, Channel::CHANNEL3}},
        {Peripherals::PE_SPI1_RX, {DMAStreamId::DMA2_Str0, Channel::CHANNEL3}},

        {Peripherals::PE_SPI2_TX, {DMAStreamId::DMA1_Str4, Channel::CHANNEL0}},
        {Peripherals::PE_SPI2_TX, {DMAStreamId::DMA1_Str6, Channel::CHANNEL9}},
        // {Peripherals::PE_SPI2_RX, {DMAStreamId::DMA1_Str1,
        // Channel::CHANNEL9}}, {Peripherals::PE_SPI2_RX,
        // {DMAStreamId::DMA1_Str3, Channel::CHANNEL0}},

        {Peripherals::PE_SPI3_TX, {DMAStreamId::DMA1_Str5, Channel::CHANNEL0}},
        {Peripherals::PE_SPI3_TX, {DMAStreamId::DMA1_Str7, Channel::CHANNEL0}},
        {Peripherals::PE_SPI3_RX, {DMAStreamId::DMA1_Str0, Channel::CHANNEL0}},
        {Peripherals::PE_SPI3_RX, {DMAStreamId::DMA1_Str2, Channel::CHANNEL0}},

        {Peripherals::PE_SPI4_TX, {DMAStreamId::DMA2_Str1, Channel::CHANNEL4}},
        {Peripherals::PE_SPI4_TX, {DMAStreamId::DMA2_Str4, Channel::CHANNEL5}},
        {Peripherals::PE_SPI4_TX, {DMAStreamId::DMA2_Str2, Channel::CHANNEL9}},
        {Peripherals::PE_SPI4_RX, {DMAStreamId::DMA2_Str0, Channel::CHANNEL4}},
        // {Peripherals::PE_SPI4_RX, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL5}},

        {Peripherals::PE_SPI5_TX, {DMAStreamId::DMA2_Str4, Channel::CHANNEL2}},
        {Peripherals::PE_SPI5_TX, {DMAStreamId::DMA2_Str6, Channel::CHANNEL7}},
        // {Peripherals::PE_SPI5_RX, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL2}},
        // {Peripherals::PE_SPI5_RX, {DMAStreamId::DMA2_Str5,
        // Channel::CHANNEL7}},
        // {Peripherals::PE_SPI5_RX, {DMAStreamId::DMA2_Str5,
        // Channel::CHANNEL9}},

        // {Peripherals::PE_SPI6_TX, {DMAStreamId::DMA2_Str5,
        // Channel::CHANNEL1}},
        {Peripherals::PE_SPI6_RX, {DMAStreamId::DMA2_Str6, Channel::CHANNEL1}},

        // {Peripherals::PE_QUADSPI, {DMAStreamId::DMA2_Str7,
        // Channel::CHANNEL3}},
        {Peripherals::PE_QUADSPI, {DMAStreamId::DMA2_Str2, Channel::CHANNEL11}},

        // UART & USART
        // {Peripherals::PE_USART1_TX,
        //  {DMAStreamId::DMA2_Str7, Channel::CHANNEL4}},
        {Peripherals::PE_USART1_RX,
         {DMAStreamId::DMA2_Str2, Channel::CHANNEL4}},
        // {Peripherals::PE_USART1_RX,
        //  {DMAStreamId::DMA2_Str5, Channel::CHANNEL4}},

        {Peripherals::PE_USART2_TX,
         {DMAStreamId::DMA1_Str6, Channel::CHANNEL4}},
        {Peripherals::PE_USART2_RX,
         {DMAStreamId::DMA1_Str5, Channel::CHANNEL4}},

        // {Peripherals::PE_USART3_TX,
        //  {DMAStreamId::DMA1_Str3, Channel::CHANNEL4}},
        {Peripherals::PE_USART3_TX,
         {DMAStreamId::DMA1_Str4, Channel::CHANNEL7}},
        // {Peripherals::PE_USART3_RX,
        //  {DMAStreamId::DMA1_Str1, Channel::CHANNEL4}},

        {Peripherals::PE_UART4_TX, {DMAStreamId::DMA1_Str4, Channel::CHANNEL4}},
        {Peripherals::PE_UART4_RX, {DMAStreamId::DMA1_Str2, Channel::CHANNEL4}},

        {Peripherals::PE_UART5_TX, {DMAStreamId::DMA1_Str7, Channel::CHANNEL4}},
        {Peripherals::PE_UART5_RX, {DMAStreamId::DMA1_Str0, Channel::CHANNEL4}},

        // {Peripherals::PE_UART7_TX, {DMAStreamId::DMA1_Str1,
        // Channel::CHANNEL5}}, {Peripherals::PE_UART7_RX,
        // {DMAStreamId::DMA1_Str3, Channel::CHANNEL5}},

        {Peripherals::PE_UART8_TX, {DMAStreamId::DMA1_Str0, Channel::CHANNEL5}},
        {Peripherals::PE_UART8_RX, {DMAStreamId::DMA1_Str6, Channel::CHANNEL5}},

        {Peripherals::PE_USART6_TX,
         {DMAStreamId::DMA2_Str6, Channel::CHANNEL5}},
        // {Peripherals::PE_USART6_TX,
        //  {DMAStreamId::DMA2_Str7, Channel::CHANNEL5}},
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
        {Peripherals::PE_I2C2_TX, {DMAStreamId::DMA1_Str4, Channel::CHANNEL8}},
        {Peripherals::PE_I2C2_RX, {DMAStreamId::DMA1_Str2, Channel::CHANNEL7}},
        // {Peripherals::PE_I2C2_RX, {DMAStreamId::DMA1_Str3,
        // Channel::CHANNEL7}},

        {Peripherals::PE_I2C3_TX, {DMAStreamId::DMA1_Str4, Channel::CHANNEL3}},
        {Peripherals::PE_I2C3_TX, {DMAStreamId::DMA1_Str0, Channel::CHANNEL8}},
        {Peripherals::PE_I2C3_RX, {DMAStreamId::DMA1_Str2, Channel::CHANNEL3}},
        // {Peripherals::PE_I2C3_RX, {DMAStreamId::DMA1_Str1,
        // Channel::CHANNEL1}},

        {Peripherals::PE_I2C4_TX, {DMAStreamId::DMA1_Str6, Channel::CHANNEL8}},
        {Peripherals::PE_I2C4_RX, {DMAStreamId::DMA1_Str2, Channel::CHANNEL2}},
        // {Peripherals::PE_I2C4_RX, {DMAStreamId::DMA1_Str1,
        // Channel::CHANNEL8}},

        // TIMERS
        // {Peripherals::PE_TIM1_UP, {DMAStreamId::DMA2_Str5,
        // Channel::CHANNEL6}},
        {Peripherals::PE_TIM1_TRIG,
         {DMAStreamId::DMA2_Str0, Channel::CHANNEL6}},
        {Peripherals::PE_TIM1_TRIG,
         {DMAStreamId::DMA2_Str4, Channel::CHANNEL6}},
        {Peripherals::PE_TIM1_COM, {DMAStreamId::DMA2_Str4, Channel::CHANNEL6}},
        {Peripherals::PE_TIM1_CH1, {DMAStreamId::DMA2_Str6, Channel::CHANNEL0}},
        {Peripherals::PE_TIM1_CH1, {DMAStreamId::DMA2_Str1, Channel::CHANNEL6}},
        // {Peripherals::PE_TIM1_CH1, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL6}},
        {Peripherals::PE_TIM1_CH2, {DMAStreamId::DMA2_Str6, Channel::CHANNEL0}},
        {Peripherals::PE_TIM1_CH2, {DMAStreamId::DMA2_Str2, Channel::CHANNEL6}},
        {Peripherals::PE_TIM1_CH3, {DMAStreamId::DMA2_Str6, Channel::CHANNEL0}},
        {Peripherals::PE_TIM1_CH3, {DMAStreamId::DMA2_Str6, Channel::CHANNEL6}},
        {Peripherals::PE_TIM1_CH4, {DMAStreamId::DMA2_Str4, Channel::CHANNEL6}},

        // {Peripherals::PE_TIM2_UP, {DMAStreamId::DMA1_Str1,
        // Channel::CHANNEL3}},
        {Peripherals::PE_TIM2_UP, {DMAStreamId::DMA1_Str7, Channel::CHANNEL3}},
        {Peripherals::PE_TIM2_CH1, {DMAStreamId::DMA1_Str5, Channel::CHANNEL3}},
        {Peripherals::PE_TIM2_CH2, {DMAStreamId::DMA1_Str6, Channel::CHANNEL3}},
        // {Peripherals::PE_TIM2_CH3, {DMAStreamId::DMA1_Str1,
        // Channel::CHANNEL3}},
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
        // Channel::CHANNEL2}},
        {Peripherals::PE_TIM4_CH3, {DMAStreamId::DMA1_Str7, Channel::CHANNEL2}},

        {Peripherals::PE_TIM5_UP, {DMAStreamId::DMA1_Str0, Channel::CHANNEL6}},
        {Peripherals::PE_TIM5_UP, {DMAStreamId::DMA1_Str6, Channel::CHANNEL6}},
        // {Peripherals::PE_TIM5_TRIG,
        //  {DMAStreamId::DMA1_Str1, Channel::CHANNEL6}},
        // {Peripherals::PE_TIM5_TRIG,
        //  {DMAStreamId::DMA1_Str3, Channel::CHANNEL6}},
        {Peripherals::PE_TIM5_CH1, {DMAStreamId::DMA1_Str2, Channel::CHANNEL6}},
        {Peripherals::PE_TIM5_CH2, {DMAStreamId::DMA1_Str4, Channel::CHANNEL6}},
        {Peripherals::PE_TIM5_CH3, {DMAStreamId::DMA1_Str0, Channel::CHANNEL6}},
        // {Peripherals::PE_TIM5_CH4, {DMAStreamId::DMA1_Str1,
        // Channel::CHANNEL6}}, {Peripherals::PE_TIM5_CH4,
        // {DMAStreamId::DMA1_Str3, Channel::CHANNEL6}},

        // {Peripherals::PE_TIM6_UP, {DMAStreamId::DMA1_Str1,
        // Channel::CHANNEL7}},

        {Peripherals::PE_TIM7_UP, {DMAStreamId::DMA1_Str2, Channel::CHANNEL1}},
        {Peripherals::PE_TIM7_UP, {DMAStreamId::DMA1_Str4, Channel::CHANNEL1}},

        {Peripherals::PE_TIM8_UP, {DMAStreamId::DMA2_Str1, Channel::CHANNEL7}},
        // {Peripherals::PE_TIM8_TRIG,
        //  {DMAStreamId::DMA2_Str7, Channel::CHANNEL7}},
        // {Peripherals::PE_TIM8_COM, {DMAStreamId::DMA2_Str7,
        // Channel::CHANNEL7}},
        {Peripherals::PE_TIM8_CH1, {DMAStreamId::DMA2_Str2, Channel::CHANNEL0}},
        {Peripherals::PE_TIM8_CH1, {DMAStreamId::DMA2_Str2, Channel::CHANNEL7}},
        {Peripherals::PE_TIM8_CH2, {DMAStreamId::DMA2_Str2, Channel::CHANNEL0}},
        // {Peripherals::PE_TIM8_CH2, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL7}},
        {Peripherals::PE_TIM8_CH3, {DMAStreamId::DMA2_Str2, Channel::CHANNEL0}},
        {Peripherals::PE_TIM8_CH3, {DMAStreamId::DMA2_Str4, Channel::CHANNEL7}},
        // {Peripherals::PE_TIM8_CH4, {DMAStreamId::DMA2_Str7,
        // Channel::CHANNEL7}},

        // Others
        {Peripherals::PE_DAC1, {DMAStreamId::DMA1_Str5, Channel::CHANNEL7}},
        {Peripherals::PE_DAC2, {DMAStreamId::DMA1_Str6, Channel::CHANNEL7}},

        {Peripherals::PE_ADC1, {DMAStreamId::DMA2_Str0, Channel::CHANNEL0}},
        {Peripherals::PE_ADC1, {DMAStreamId::DMA2_Str4, Channel::CHANNEL0}},

        {Peripherals::PE_ADC2, {DMAStreamId::DMA2_Str2, Channel::CHANNEL1}},
        // {Peripherals::PE_ADC2, {DMAStreamId::DMA2_Str3, Channel::CHANNEL1}},

        {Peripherals::PE_ADC3, {DMAStreamId::DMA2_Str0, Channel::CHANNEL2}},
        {Peripherals::PE_ADC3, {DMAStreamId::DMA2_Str1, Channel::CHANNEL2}},

        {Peripherals::PE_SAI1_A, {DMAStreamId::DMA2_Str1, Channel::CHANNEL0}},
        {Peripherals::PE_SAI1_A, {DMAStreamId::DMA2_Str6, Channel::CHANNEL10}},
        // {Peripherals::PE_SAI1_A, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL0}},
        {Peripherals::PE_SAI2_A, {DMAStreamId::DMA2_Str4, Channel::CHANNEL3}},
        {Peripherals::PE_SAI2_A, {DMAStreamId::DMA2_Str2, Channel::CHANNEL10}},

        // {Peripherals::PE_SAI1_B, {DMAStreamId::DMA2_Str5,
        // Channel::CHANNEL0}},
        {Peripherals::PE_SAI1_B, {DMAStreamId::DMA2_Str4, Channel::CHANNEL1}},
        {Peripherals::PE_SAI1_B, {DMAStreamId::DMA2_Str0, Channel::CHANNEL10}},
        // {Peripherals::PE_SAI2_B, {DMAStreamId::DMA2_Str7,
        // Channel::CHANNEL0}},
        {Peripherals::PE_SAI2_B, {DMAStreamId::DMA2_Str6, Channel::CHANNEL3}},
        {Peripherals::PE_SAI2_B, {DMAStreamId::DMA2_Str1, Channel::CHANNEL10}},

        {Peripherals::PE_DCMI, {DMAStreamId::DMA2_Str1, Channel::CHANNEL1}},
        // {Peripherals::PE_DCMI, {DMAStreamId::DMA2_Str7, Channel::CHANNEL1}},

        // {Peripherals::PE_SDMMC1, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL4}},
        {Peripherals::PE_SDMMC1, {DMAStreamId::DMA2_Str6, Channel::CHANNEL4}},
        {Peripherals::PE_SDMMC2, {DMAStreamId::DMA2_Str0, Channel::CHANNEL11}},
        // {Peripherals::PE_SDMMC2, {DMAStreamId::DMA2_Str5,
        // Channel::CHANNEL11}},

        // {Peripherals::PE_CRYP_OUT, {DMAStreamId::DMA2_Str5,
        // Channel::CHANNEL2}},
        {Peripherals::PE_CRYP_IN, {DMAStreamId::DMA2_Str6, Channel::CHANNEL2}},

        // {Peripherals::PE_HASH_IN, {DMAStreamId::DMA2_Str7,
        // Channel::CHANNEL2}},

        // {Peripherals::PE_SPDIFRX_DT,
        //  {DMAStreamId::DMA1_Str1, Channel::CHANNEL0}},
        {Peripherals::PE_SPDIFRX_CS,
         {DMAStreamId::DMA1_Str6, Channel::CHANNEL0}},

        {Peripherals::PE_DFSDM1_FLT0,
         {DMAStreamId::DMA2_Str0, Channel::CHANNEL8}},
        {Peripherals::PE_DFSDM1_FLT0,
         {DMAStreamId::DMA2_Str4, Channel::CHANNEL8}},
        {Peripherals::PE_DFSDM1_FLT1,
         {DMAStreamId::DMA2_Str1, Channel::CHANNEL8}},
        // {Peripherals::PE_DFSDM1_FLT1,
        //  {DMAStreamId::DMA2_Str5, Channel::CHANNEL8}},
        {Peripherals::PE_DFSDM1_FLT2,
         {DMAStreamId::DMA2_Str2, Channel::CHANNEL8}},
        {Peripherals::PE_DFSDM1_FLT2,
         {DMAStreamId::DMA2_Str6, Channel::CHANNEL8}},
        // {Peripherals::PE_DFSDM1_FLT3, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL8}},
        // {Peripherals::PE_DFSDM1_FLT3,
        //  {DMAStreamId::DMA2_Str7, Channel::CHANNEL8}},

        {Peripherals::PE_JPEG_IN, {DMAStreamId::DMA2_Str0, Channel::CHANNEL9}},
        // {Peripherals::PE_JPEG_IN, {DMAStreamId::DMA2_Str3,
        // Channel::CHANNEL9}}
        {Peripherals::PE_JPEG_OUT, {DMAStreamId::DMA2_Str1, Channel::CHANNEL9}},
        {Peripherals::PE_JPEG_OUT, {DMAStreamId::DMA2_Str4, Channel::CHANNEL9}},
};

}  // namespace DMADefs

}  // namespace Boardcore
