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

namespace Boardcore
{

InternalADC::InternalADC(ADC_TypeDef& ADCx_, const float V_SUPPLY_,
                         const bool isUsingDMA_,
                         DMA_Stream_TypeDef* DMAx_Streamx_)
    : ADCx(ADCx_), V_SUPPLY(V_SUPPLY_), isUsingDMA(isUsingDMA_),
      DMAx_Streamx(DMAx_Streamx_)
{
    resetRegisters();
    enableADCClock();

    // Init indexMap
    for (auto i = 0; i < CH_NUM; i++)
    {
        indexMap[i] = -1;
    }
}

InternalADC::~InternalADC()
{
    resetRegisters();
    disableADCClock();
}

bool InternalADC::init()
{
    // Turn on the ADC
    ADCx.CR2 |= ADC_CR2_ADON;

    // Set single conversion mode
    ADCx.CR2 &= ~ADC_CR2_CONT;

    // Set scan mode
    ADCx.CR1 |= ADC_CR1_SCAN;

    // Data alignment
    ADCx.CR2 &= ~ADC_CR2_ALIGN;  // right

    if (isUsingDMA)
    {
        // Set the DMA peripheral address
        DMAx_Streamx->PAR = (uint32_t) & (ADCx.DR);

        // Set the DMA memory address
        DMAx_Streamx->M0AR = (uint32_t)values;

        // Enable DMA on ADC
        ADCx.CR2 |= ADC_CR2_DMA;

        // Enable DMA stream
        DMAx_Streamx->CR |= DMA_SxCR_EN;

        // Check if we are using the DMA2 controller, otherwise it's DMA1
        if (((uint32_t)&DMAx_Streamx & ~0xFF) == (uint32_t)DMA2_BASE)
        {
            DMAx = DMA2;
        }

        // Find the DMA stream number
        streamNum = ((((uint32_t)&DMAx_Streamx) & 0xFF) - 0x10) / 0x18;

        // Check which registers to use
        statusReg    = (streamNum < 4 ? &(DMAx->LISR) : &(DMAx->HISR));
        clearFlagReg = (streamNum < 4 ? &(DMAx->LIFCR) : &(DMAx->HIFCR));

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

        // All this is because the status bits are not stored in the
        // DMAx_Streamx registers but in the main DMA controller status
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
        DMAx_Streamx->NDTR = activeChannels;
    }

    // Set channel's sample time
    setChannelSampleTime(channel, sampleTime);

    return true;
}

ADCData InternalADC::getVoltage(Channel channel)
{
    float voltage = 0;

    if (indexMap[channel] >= 0 && indexMap[channel] < CH_NUM)
    {
        voltage = values[indexMap[channel]] * V_SUPPLY / RESOLUTION;
    }

    return ADCData{timestamp, (uint8_t)channel, voltage};
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

ADCData InternalADC::sampleImpl()
{
    if (!isUsingDMA)
    {
        startInjectedConversion();

        // Wait for end of conversion
        while (!(ADCx.SR & ADC_SR_JEOC))
            ;

        // Read all 4 channels (faster than read only the enabled ones)
        values[0] = ADCx.JDR1;
        values[1] = ADCx.JDR2;
        values[2] = ADCx.JDR3;
        values[3] = ADCx.JDR4;
    }
    else
    {
        // Rewrite the DMA bit in ADC CR2 (reference manual chapter 13.8.1)
        ADCx.CR2 &= ~ADC_CR2_DMA;
        ADCx.CR2 |= ADC_CR2_DMA;

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

    timestamp = TimestampTimer::getInstance().getTimestamp();

    return lastSample;
}

inline void InternalADC::resetRegisters()
{
    // Reset the ADC configuration
    ADCx.CR1   = 0;
    ADCx.CR2   = 0;
    ADCx.SMPR1 = 0;
    ADCx.SMPR2 = 0;
    ADCx.JOFR1 = 0;
    ADCx.JOFR2 = 0;
    ADCx.JOFR3 = 0;
    ADCx.JOFR4 = 0;
    ADCx.HTR   = 0;
    ADCx.LTR   = 0;
    ADCx.SQR1  = 0;
    ADCx.SQR2  = 0;
    ADCx.SQR3  = 0;
    ADCx.JSQR  = 0;
}

inline void InternalADC::startInjectedConversion()
{
    ADCx.CR2 |= ADC_CR2_JSWSTART;
}

inline void InternalADC::startRegularConversion()
{
    ADCx.CR2 |= ADC_CR2_SWSTART;
}

inline bool InternalADC::addInjectedChannel(Channel channel)
{
    // Check active channels number
    if (activeChannels >= 4)
    {
        return false;
    }

    // Add the channel to the sequence, starting from position 4
    ADCx.JSQR |= channel << (15 - activeChannels * 5);

    // Update the channels number in the register
    ADCx.JSQR &= 0x000FFFFF;
    ADCx.JSQR |= activeChannels << 20;

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
            sqrPtr = &(ADCx.SQR3);
            break;
        case 2:
            sqrPtr = &(ADCx.SQR2);
            break;
        default:
            sqrPtr = &(ADCx.SQR1);
    }
    *sqrPtr = channel << ((activeChannels % 6) * 5);

    // Update the channels number in the register
    ADCx.SQR1 |= activeChannels << 20;

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
        smprPtr = &(ADCx.SMPR2);
    }
    else
    {
        smprPtr = &(ADCx.SMPR1);
    }
    *smprPtr = sampleTime << (channel * 3);
}

inline void InternalADC::enableADCClock()
{
    miosix::FastInterruptDisableLock dLock;

    if (&ADCx == ADC1)
    {
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    }
    else if (&ADCx == ADC2)
    {
        RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
    }
    else if (&ADCx == ADC3)
    {
        RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
    }

    RCC_SYNC();
}

inline void InternalADC::disableADCClock()
{
    miosix::FastInterruptDisableLock dLock;

    if (&ADCx == ADC1)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
    }
    else if (&ADCx == ADC2)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC2EN;
    }
    else if (&ADCx == ADC3)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC3EN;
    }
}

}  // namespace Boardcore
