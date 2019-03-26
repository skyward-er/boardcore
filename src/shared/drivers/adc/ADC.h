/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Matteo Michele Piazzolla, Luca Erbetta
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
#pragma once
#include "Common.h"

/**
 * @brief Driver for stm32 internal ADC.
 * Allows for conversion on multiple channels, one at a time (no SCAN mode),
 * with per-channel sample time.
 * This class is not thread-safe. All conversions must be executed on the same
 * thread.
 * @tparam ADC_num Which ADC to use: [1,2,3] on stm32f429zi
 */
template <unsigned ADC_num>
class SensorADC
{
public:
    /**
     * @brief One ADC channel
     */
    enum class Channel : uint8_t
    {
        CH1 = 1,
        CH2,
        CH3,
        CH4,
        CH5,
        CH6,
        CH7,
        CH8,
        CH9,
        CH10,
        CH11,
        CH12,
        CH13,
        CH14,
        CH15,
        CH16,
        CH17,
        CH18,
        CH19
    };

    /**
     * @brief Conversion sample time. See datasheet.
     * 
     */
    enum class SampleTime : uint8_t
    {
        CYCLES_3   = 0x0,
        CYCLES_15  = 0x1,
        CYCLES_28  = 0x2,
        CYCLES_56  = 0x3,
        CYCLES_84  = 0x4,
        CYCLES_112 = 0x5,
        CYCLES_144 = 0x6,
        CYCLES_480 = 0x7
    };

    SensorADC()
    {
        adc = getADC(ADC_num);
        enableADC();

        adc->SQR1 = 0x0;  // One conversion

        adc->CR2 = ADC_CR2_ADON;
        // CCR pre scaler
        ADC->CCR = 0x3 << 16;
    }

    ~SensorADC() { disableADC(); }

    /**
     * @brief Converts the specified channel
     * 
     * @param channel The channel to convert
     * @param sample_time Conversion sample time
     * @return uint16_t Measured value (12 bits)
     */
    uint16_t convertChannel(Channel channel,
                            SampleTime sample_time = SampleTime::CYCLES_15)
    {
        // Select the channel
        adc->SQR3 = static_cast<uint32_t>(channel);
        // Set sample time
        adc->SMPR2 = static_cast<uint32_t>(sample_time);

        // Start conversion
        adc->CR2 |= ADC_CR2_SWSTART;

        // Wait for conversion to finish
        const int TIMEOUT = 50;  // Make sure the loop terminates eventually
        int i             = 0;
        while ((adc->SR & ADC_SR_EOC) == 0 && i < TIMEOUT)
        {
            ++i;
        }
        adc->SR = 0; // Clear the status register

        // Conversion terminated, read result
        return static_cast<uint16_t>(adc->DR & 0x0000FFFF);
    }

private:
    void enableADC()
    {
        miosix::FastInterruptDisableLock dLock;

        if (adc == ADC1)
        {
            RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
            printf("ADC1\n");
        }
        else if (adc == ADC2)
        {
            RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
            printf("ADC2\n");
        }
        else if (adc == ADC3)
        {
            RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
            printf("ADC3\n");
        }

        RCC_SYNC();
    }

    void disableADC()
    {
        miosix::FastInterruptDisableLock dLock;

        if (adc == ADC1)
        {
            RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
        }
        else if (adc == ADC2)
        {
            RCC->APB2ENR &= ~RCC_APB2ENR_ADC2EN;
        }
        else if (adc == ADC3)
        {
            RCC->APB2ENR &= ~RCC_APB2ENR_ADC3EN;
        }

        RCC_SYNC();
    }

    constexpr ADC_TypeDef* getADC(unsigned n)
    {
        return n == 1 ? ADC1 : n == 2 ? ADC2 : ADC3;
    }

    ADC_TypeDef* adc;
};
