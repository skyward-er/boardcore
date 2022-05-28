/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include "InternalADC.h"

#include <drivers/timer/TimestampTimer.h>
#include <utils/ClockUtils.h>

namespace Boardcore
{

InternalADC::InternalADC(ADC_TypeDef* adc, const float supplyVoltage,
                         const bool isUsingDMA, DMA_Stream_TypeDef* dmaStream)
    : adc(adc), supplyVoltage(supplyVoltage), isUsingDMA(isUsingDMA),
      dmaStream(dmaStream)
{
    resetRegisters();
    ClockUtils::enablePeripheralClock(adc);

    if (isUsingDMA)
    {
        ClockUtils::enablePeripheralClock(dma);

        // Find the DMA stream number
        streamNum = ((((uint32_t)&dmaStream) & 0xFF) - 0x10) / 0x18;

        // Check which registers to use
        statusReg    = (streamNum < 4 ? &(dma->LISR) : &(dma->HISR));
        clearFlagReg = (streamNum < 4 ? &(dma->LIFCR) : &(dma->HIFCR));

        // Create the masks for the status bits (this will do for both high and
        // low registers as well as status and reset status registers)
        switch (streamNum % 4)
        {
            case 0:
                transferCompleteMask = DMA_LISR_TCIF0;
                transferErrorMask    = DMA_LISR_TEIF0;
                break;
            case 1:
                transferCompleteMask = DMA_LISR_TCIF1;
                transferErrorMask    = DMA_LISR_TEIF1;
                break;
            case 2:
                transferCompleteMask = DMA_LISR_TCIF2;
                transferErrorMask    = DMA_LISR_TEIF2;
                break;
            case 3:
                transferCompleteMask = DMA_LISR_TCIF3;
                transferErrorMask    = DMA_LISR_TEIF3;
                break;
        }
    }

    // Init indexMap
    for (auto i = 0; i < CH_NUM; i++)
    {
        indexMap[i] = -1;
    }
}

InternalADC::~InternalADC()
{
    resetRegisters();
    ClockUtils::disablePeripheralClock(adc);
    // Do not disable the DMA controller, other streams could use it!
}

bool InternalADC::init()
{
    // Turn on the ADC
    adc->CR2 |= ADC_CR2_ADON;

    // Set single conversion mode
    adc->CR2 &= ~ADC_CR2_CONT;

    // Set scan mode
    adc->CR1 |= ADC_CR1_SCAN;

    // Data alignment
    adc->CR2 &= ~ADC_CR2_ALIGN;  // right

    if (isUsingDMA)
    {
        // Set the DMA peripheral address
        dmaStream->PAR = (uint32_t) & (adc->DR);

        // Set the DMA memory address
        dmaStream->M0AR = (uint32_t)values;

        // Enable DMA on ADC
        adc->CR2 |= ADC_CR2_DMA;

        // Enable DMA stream
        dmaStream->CR |= DMA_SxCR_EN;

        // Check if we are using the DMA2 controller, otherwise it's DMA1
        if (((uint32_t)&dmaStream & ~0xFF) == (uint32_t)DMA2_BASE)
        {
            dma = DMA2;
        }

        // All this is because the status bits are not stored in the
        // dmaStream registers but in the main DMA controller status
        // register. We could not care about the transfer status but the
        // timestamp would not match the values read after calling sample().

        // We calculate this values to avoid doing it every time in
        // sampleImpl().
    }

    return true;
}

bool InternalADC::enableChannel(Channel channel)
{
    return enableChannel(channel, CYCLES_3);
}

bool InternalADC::enableChannel(Channel channel, SampleTime sampleTime)
{
    // Check channel number
    if (channel < CH0 || channel >= CH_NUM)
    {
        return false;
    }

    // Add channel to the sequence
    if (!isUsingDMA)
    {
        if (!addInjectedChannel(channel))
        {
            return false;
        }
    }
    else
    {
        if (!addRegularChannel(channel))
        {
            return false;
        }

        // Update the DMA number of data
        dmaStream->NDTR = activeChannels;
    }

    // Set channel's sample time
    setChannelSampleTime(channel, sampleTime);

    return true;
}

InternalADCData InternalADC::getVoltage(Channel channel)
{
    float voltage = 0;

    if (indexMap[channel] >= 0 && indexMap[channel] < CH_NUM)
    {
        voltage = values[indexMap[channel]] * supplyVoltage / RESOLUTION;
    }

    return InternalADCData{timestamp, (uint8_t)channel, voltage};
}

bool InternalADC::selfTest()
{
    // Try a single sample and check for error
    sample();

    if (lastError != NO_ERRORS)
    {
        return false;
    }

    return true;
}

InternalADCData InternalADC::sampleImpl()
{
    if (!isUsingDMA)
    {
        startInjectedConversion();

        // Wait for end of conversion
        while (!(adc->SR & ADC_SR_JEOC))
            ;

        // Read all 4 channels (faster than read only the enabled ones)
        values[0] = adc->JDR1;
        values[1] = adc->JDR2;
        values[2] = adc->JDR3;
        values[3] = adc->JDR4;
    }
    else
    {
        // Rewrite the DMA bit in ADC CR2 (reference manual chapter 13.8.1)
        adc->CR2 &= ~ADC_CR2_DMA;
        adc->CR2 |= ADC_CR2_DMA;

        startRegularConversion();

        // This should trigger the DMA stream for each channel's conversion

        // Wait for tranfer end
        while (!(*statusReg & (transferCompleteMask | transferErrorMask)))
            ;

        // Clear the transfer complete flag
        *clearFlagReg |= transferCompleteMask;

        // If and error has occurred (probaly due to a higher priority stream)
        // don't update the timestamp, the values should not have been updated
        if (*statusReg & transferErrorMask)
        {
            // Clear the transfer error flag
            *clearFlagReg |= transferErrorMask;

            // Signal an error
            lastError = DMA_ERROR;
            return lastSample;
        }
    }

    timestamp = TimestampTimer::getTimestamp();

    return lastSample;
}

inline void InternalADC::resetRegisters()
{
    // Reset the ADC configuration
    adc->CR1   = 0;
    adc->CR2   = 0;
    adc->SMPR1 = 0;
    adc->SMPR2 = 0;
    adc->JOFR1 = 0;
    adc->JOFR2 = 0;
    adc->JOFR3 = 0;
    adc->JOFR4 = 0;
    adc->HTR   = 0;
    adc->LTR   = 0;
    adc->SQR1  = 0;
    adc->SQR2  = 0;
    adc->SQR3  = 0;
    adc->JSQR  = 0;
}

inline void InternalADC::startInjectedConversion()
{
    adc->CR2 |= ADC_CR2_JSWSTART;
}

inline void InternalADC::startRegularConversion()
{
    adc->CR2 |= ADC_CR2_SWSTART;
}

inline bool InternalADC::addInjectedChannel(Channel channel)
{
    // Check active channels number
    if (activeChannels >= 4)
    {
        return false;
    }

    // Add the channel to the sequence, starting from position 4
    adc->JSQR |= channel << (15 - activeChannels * 5);

    // Update the channels number in the register
    adc->JSQR &= 0x000FFFFF;
    adc->JSQR |= activeChannels << 20;

    // Increment the index of all enabled channels
    for (auto i = 0; i < CH_NUM; i++)
    {
        if (indexMap[i] >= 0)
        {
            indexMap[i]++;
        }
    }

    // Set this channel index to 0
    indexMap[channel] = 0;

    // Update the counter
    activeChannels++;

    return true;
}

inline bool InternalADC::addRegularChannel(Channel channel)
{
    // Check active channels number
    if (activeChannels >= 16)
    {
        return false;
    }

    // Add the channel to the sequence
    volatile uint32_t* sqrPtr;
    switch (activeChannels % 6)
    {
        case 1:
            sqrPtr = &(adc->SQR3);
            break;
        case 2:
            sqrPtr = &(adc->SQR2);
            break;
        default:
            sqrPtr = &(adc->SQR1);
    }
    *sqrPtr = channel << ((activeChannels % 6) * 5);

    // Update the channels number in the register
    adc->SQR1 |= activeChannels << 20;

    // Save the index of the channel in the ADC's regular sequence
    indexMap[channel] = activeChannels;

    // Update the counter
    activeChannels++;

    return true;
}

inline void InternalADC::setChannelSampleTime(Channel channel,
                                              SampleTime sampleTime)
{
    volatile uint32_t* smprPtr;
    if (channel <= 9)
    {
        smprPtr = &(adc->SMPR2);
    }
    else
    {
        smprPtr = &(adc->SMPR1);
    }
    *smprPtr = sampleTime << (channel * 3);
}

}  // namespace Boardcore
