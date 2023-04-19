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

#if defined(STM32F407xx) || defined(STM32F429xx)
#define TEMP30_CAL_VALUE ((uint16_t*)((uint32_t)0x1FFF7A2C))
#define TEMP110_CAL_VALUE ((uint16_t*)((uint32_t)0x1FFF7A2E))
#define TEMP30 30
#define TEMP110 110
#elif defined(STM32F767xx) || defined(STM32F769xx)
#define TEMP30_CAL_VALUE ((uint16_t*)((uint32_t)0x1FF0F44C))
#define TEMP110_CAL_VALUE ((uint16_t*)((uint32_t)0x1FF0F44E))
#define TEMP30 30
#define TEMP110 110
#else
#warning This micro controller does not have a calibrated temperature sensor or is not currently supported by this driver
#define WITHOUT_CALIBRATION
#endif

#if defined(STM32F407xx) || defined(STM32F205xx)
#define AUX_CH InternalADC::CH16
#define VBAT_DIV 2.0f
#elif defined(STM32F429xx) || defined(STM32F767xx) || defined(STM32F769xx)
#define AUX_CH InternalADC::CH18
#define VBAT_DIV 4.0f
#endif

namespace Boardcore
{

InternalADC::InternalADC(ADC_TypeDef* adc, const float supplyVoltage)
    : adc(adc), supplyVoltage(supplyVoltage)
{
    resetRegisters();
    ClockUtils::enablePeripheralClock(adc);

    for (int i = 0; i < CH_NUM; i++)
    {
        channelsEnabled[i]   = false;
        channelsRawValues[i] = 0;
    }
}

InternalADC::~InternalADC()
{
    resetRegisters();
    ClockUtils::disablePeripheralClock(adc);
}

bool InternalADC::init()
{
    // Turn on the ADC
    adc->CR2 |= ADC_CR2_ADON;

    return true;
}

bool InternalADC::selfTest() { return true; }

InternalADCData InternalADC::sampleImpl()
{
    for (int i = 0; i < CH_NUM; i++)
    {
        if (channelsEnabled[i])
        {
            channelsRawValues[i] = readChannel(static_cast<Channel>(i));
        }
    }

    // Prepare the auxiliary channel for the next sample
    if (vbatLastRead)
    {
        vbatVoltageRawValue = channelsRawValues[AUX_CH];

        if (tempEnabled)
        {
            ADC->CCR &= ~ADC_CCR_VBATE;
            ADC->CCR |= ADC_CCR_TSVREFE;
            vbatLastRead = false;
        }
    }
    else
    {
        temperatureRawValue = channelsRawValues[AUX_CH];

        if (vbatEnabled)
        {
            ADC->CCR |= ADC_CCR_VBATE;
            ADC->CCR &= ~ADC_CCR_TSVREFE;
            vbatLastRead = true;
        }
    }

    timestamp = TimestampTimer::getTimestamp();

    return lastSample;
}

void InternalADC::enableChannel(Channel channel, SampleTime sampleTime)
{
    channelsEnabled[channel] = true;

    setChannelSampleTime(channel, sampleTime);
}

void InternalADC::disableChannel(Channel channel)
{
    channelsEnabled[channel]   = false;
    channelsRawValues[channel] = 0;
}

void InternalADC::enableTemperature(SampleTime sampleTime)
{
    tempEnabled = true;

    ADC->CCR &= ~ADC_CCR_VBATE;
    ADC->CCR |= ADC_CCR_TSVREFE;
    vbatLastRead = false;

    enableChannel(AUX_CH, sampleTime);
}

void InternalADC::disableTemperature() { tempEnabled = false; }

void InternalADC::enableVbat(SampleTime sampleTime)
{
    vbatEnabled = true;

    ADC->CCR |= ADC_CCR_VBATE;
    ADC->CCR &= ~ADC_CCR_TSVREFE;
    vbatLastRead = true;

    enableChannel(AUX_CH, sampleTime);
}

void InternalADC::disableVbat() { vbatEnabled = false; }

InternalADCData InternalADC::getVoltage(Channel channel)
{
    return {timestamp, channel,
            channelsRawValues[channel] * supplyVoltage / RESOLUTION};
}

TemperatureData InternalADC::getTemperature()
{
    TemperatureData data;
    data.temperatureTimestamp = timestamp;
    data.temperature = temperatureRawValue * supplyVoltage / RESOLUTION;

#ifdef WITHOUT_CALIBRATION
    // Default conversion
    data.temperature = ((data.temperature - 0.76) / 0.0025) + 25;
#else
    // Factory calibration
    // Read "Temperature sensor characteristics" chapter in the datasheet
    float voltage30  = static_cast<float>(*TEMP30_CAL_VALUE) * 3.3 / 4095;
    float voltage110 = static_cast<float>(*TEMP110_CAL_VALUE) * 3.3 / 4095;
    float slope      = (voltage110 - voltage30) / (TEMP110 - TEMP30);
    data.temperature = ((data.temperature - voltage30) / slope) + TEMP30;
#endif

    return data;
}

InternalADCData InternalADC::getVbatVoltage()
{
    return {timestamp, AUX_CH,
            vbatVoltageRawValue * supplyVoltage / RESOLUTION * VBAT_DIV};
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

inline void InternalADC::setChannelSampleTime(Channel channel,
                                              SampleTime sampleTime)
{
    if (channel <= 9)
    {
        adc->SMPR2 &= ~(0x7 << (channel * 3));
        adc->SMPR2 |= sampleTime << (channel * 3);
    }
    else
    {
        adc->SMPR1 &= ~(0x7 << ((channel - 10) * 3));
        adc->SMPR1 |= sampleTime << ((channel - 10) * 3);
    }
}

float InternalADC::readChannel(Channel channel)
{
    // Select channel
    adc->SQR3 = channel;

    // Start conversion
    adc->CR2 |= ADC_CR2_SWSTART;

    while (!(adc->SR & ADC_SR_EOC))
        ;

    return static_cast<uint16_t>(adc->DR);
}

}  // namespace Boardcore
