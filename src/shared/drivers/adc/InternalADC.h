/*
 * Copyright (c) 2015-2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Matteo Michele Piazzolla
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

#include <cassert>
#include <utility>
#include <vector>
#include "Common.h"
#include "Debug.h"
#include "InternalADCData.h"

using std::pair;
using std::vector;

template <unsigned ADC_n, class GpioPin>
class SensorADC
{
public:
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

    SensorADC() : adc(getADC(ADC_n)) { enableADC(); }

    ~SensorADC() { disableADC(); }

    bool enableChannel(Channel ch, SampleTime sample_time)
    {
        if (channels.size() == 16)
        {
            ruturn false;
        }

        for (auto& i : channels)
        {
            if (ch.first == i)
            {
                // channel already enabled
                return false;
            }
        }

        channels.push_back({ch, sample_time});

        updateChannels();

        return true;
    }

    void disableChannel(Channel ch)
    {
        for (auto it = channels.begin(); it < channels.end(); ++it)
        {
            if ((*it).first == ch)
            {
                channels.erase(it);

                updateChannels();
                return;
            }
        }
    }

private:
    constexpr ADC_TypeDef* getADC(unsigned n)
    {
        return n == 1 ? ADC1 : n == 2 ? ADC2 : ADC3;
    }

    static void enableADC()
    {
        miosix::FastInterruptDisableLock dLock;

        if (adc == ADC1)
        {
            RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
            TRACE("ADC1\n");
        }
        else if (adc == ADC2)
        {
            RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
            TRACE("ADC2\n");
        }
        else if (adc == ADC3)
        {
            RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
            TRACE("ADC3\n");
        }

        RCC_SYNC();
    }

    static inline void disableADC()
    {
        miosix::FastInterruptDisableLock dLock;

        if (adc == ADC1)
        {
            RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
            TRACE("ADC1\n");
        }
        else if (adc == ADC2)
        {
            RCC->APB2ENR &= ~RCC_APB2ENR_ADC2EN;
            TRACE("ADC2\n");
        }
        else if (adc == ADC3)
        {
            RCC->APB2ENR &= ~RCC_APB2ENR_ADC3EN;
            TRACE("ADC3\n");
        }

        RCC_SYNC();
    }

    void updateChannels()
    {
        // Clear the registers
        adc->SQR1 = 0;
        adc->SQR2 = 0;
        adc->SQR3 = 0;
        adc->SMPR1 = 0;
        adc->SMPR2 = 0;

        if(channels.size() == 0)
        {
            // If no channel is enabled, clear the registers and return.
            return;   
        }
        // Store the channel sequence in the SQRx registers
        uint8_t L = channels.size() - 1;

        uint32_t* sqr_ptr =
            &(adc->SQR3);  // Pointer to the SQR3 register, defining the first 6
                           // channels in the sequence. If more channels are to
                           // be converted, this pointer will be updated to
                           // point the register that stores the sequence for
                           // subsequent channels.

        unsigned int offset = 0;
        for (unsigned int i = 0; i < channels.size(); i++)
        {
            if (i == 6)
            {
                sqr_ptr = SQR2;
                offset  = 6;
            }
            else if (i == 12)
            {
                sqr_ptr = SQR1;
                offset  = 12;
            }

            *sqr_ptr |= (static_cast<uint8_t>(channels[i].first) - 1) 
                                        << ((i - offset)*5;
        }

        adc->SQR1 |= L << 20;

        // Now update sample times for each channel

        uint32_t* SMPR;
        for(auto& i : channels)
        {
            uint8_t ch = static_cast<uint8_t>(i.first) - 1; // CH1 --> 0
            uint8_t sample_time = static_cast<uint8_t>(i.second);
            if(ch >= 10)
            {
                SMPR = &(adc->SMPR1);
                offset = 10;
            }else
            {
                SMPR = &(adc->SMPR1);
                offset = 0;
            }  
            
            *SMPR |= static_cast<uint32_t>(sample_time) << ((ch - offset)*3)
        }
    }

    ADC_TypeDef* adc;
    // Channels to convert, in order of conversion. size() must not exceed 16.
    vector<Pair<Channel, SampleTime>> channels;
};