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

static constexpr int ADC_RESOLUTION = 4095;

namespace Boardcore
{

#if defined(STM32F407xx) || defined(STM32F429xx)
#define CAL_PT1_VALUE ((uint16_t volatile *)((uint32_t)0x1FFF7A2C))
#define CAL_PT2_VALUE ((uint16_t volatile *)((uint32_t)0x1FFF7A2E))
static const float CAL_PT1_TEMP = 30;
static const float CAL_PT2_TEMP = 110;
static const float CAL_V_DDA    = 3.3f;
#elif defined(STM32F767xx) || defined(STM32F769xx) || defined(STM32F756xx)
#define CAL_PT1_VALUE ((uint16_t volatile *)((uint32_t)0x1FF0F44C))
#define CAL_PT2_VALUE ((uint16_t volatile *)((uint32_t)0x1FF0F44E))
static const float CAL_PT1_TEMP           = 30;
static const float CAL_PT2_TEMP           = 110;
static const float CAL_V_DDA              = 3.3f;
#else
#warning This micro controller does not have a calibrated temperature sensor or is not currently supported by this driver
#endif

#if defined(STM32F407xx) || defined(STM32F205xx)
static const InternalADC::Channel TEMP_CH = InternalADC::CH16;
static const InternalADC::Channel VBAT_CH = InternalADC::CH18;
static const float VBAT_DIV               = 2.0f;
#elif defined(STM32F429xx) || defined(STM32F767xx) || defined(STM32F769xx) || \
    defined(STM32F756xx)
static const InternalADC::Channel TEMP_CH = InternalADC::CH18;
static const InternalADC::Channel VBAT_CH = InternalADC::CH18;
static const float VBAT_DIV               = 4.0f;
#endif

// Error the user if the current target is missing the V_DDA_VOLTAGE macro
// If it is missing you need to define it, preferably in the board_settings.h
// file in miosix. Check your board schematic to find the voltage value.
#ifndef V_DDA_VOLTAGE
#warning Missing V_DDA_VOLTAGE definition for current target, using default value of 3.0V
#define V_DDA_VOLTAGE 3.0f
#endif

InternalADC::InternalADC(ADC_TypeDef *adc) : adc(adc)
{
#ifndef INTERNAL_ADC_WITHOUT_CALIBRATION
    loadCalibrationValues();
#endif

    resetRegisters();
    ClockUtils::enablePeripheralClock(adc);

    // Set the clock divider for the analog circuitry to the highest value (/8).
    // Currently there is no need to speed up ADC reading. For this reason we
    // use the safest setting.
    // If it will need to be changed you need to check the datasheet for the
    // maximum frequency the analog circuitry supports and compare it with the
    // parent clock (APB2). Also you need to take into account the sampling time
    // for the temperature sensor.
    ADC->CCR |= ADC_CCR_ADCPRE_1 | ADC_CCR_ADCPRE_0;

    for (int i = 0; i < CH_NUM; i++)
    {
        channelsEnabled[i] = false;
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
    InternalADCData newData;
    newData.timestamp = TimestampTimer::getTimestamp();

    for (int i = 0; i < CH16; i++)
    {
        if (channelsEnabled[i])
        {
            newData.voltage[i] = readChannel(static_cast<Channel>(i));
            newData.voltage[i] =
                newData.voltage[i] * V_DDA_VOLTAGE / ADC_RESOLUTION;
        }
    }

    /**
     * Quirk: the temperature and vbat sensors are enabled and then disabled. If
     * left enabled they somehow disrupt other channels measurements. I did not
     * find description of this behaviour anywhere but observed it during
     * testing. Also the temperature sensors has a startup time of 10us. 12us is
     * used because during test 10us were not enough.
     * The startup time is the same for all Boardcore supported micros.
     */

    if (tempEnabled)
    {
        ADC->CCR |= ADC_CCR_TSVREFE;
        miosix::delayUs(12);  // Temperature sensor startup time
        auto temperatureRawValue = readChannel(static_cast<Channel>(TEMP_CH));
        ADC->CCR &= ~ADC_CCR_TSVREFE;

        // Conversion
        if (temperatureRawValue != 0)
        {
            newData.temperature =
                temperatureRawValue * V_DDA_VOLTAGE / ADC_RESOLUTION;

#ifdef INTERNAL_ADC_WITHOUT_CALIBRATION
            // Default conversion
            newData.temperature = ((newData.temperature - 0.76) / 0.0025) + 25;
#else
            // Factory calibration
            newData.temperature = newData.temperature - calPt1Voltage;
            newData.temperature /= calSlope;
            newData.temperature += CAL_PT1_TEMP;
#endif
        }
        else
        {
            newData.temperature = 0;
        }
    }

    if (vbatEnabled)
    {
        ADC->CCR |= ADC_CCR_VBATE;
        auto vbatVoltageRawValue = readChannel(static_cast<Channel>(VBAT_CH));
        ADC->CCR &= ~ADC_CCR_VBATE;

        newData.vBat =
            vbatVoltageRawValue * V_DDA_VOLTAGE / ADC_RESOLUTION * VBAT_DIV;
    }

    return newData;
}

void InternalADC::enableChannel(Channel channel, SampleTime sampleTime)
{
    channelsEnabled[channel] = true;

    setChannelSampleTime(channel, sampleTime);
}

void InternalADC::disableChannel(Channel channel)
{
    channelsEnabled[channel] = false;
}

void InternalADC::enableTemperature(SampleTime sampleTime)
{
    tempEnabled = true;
    enableChannel(TEMP_CH, sampleTime);
}

void InternalADC::disableTemperature()
{
    tempEnabled = false;
    if (!vbatEnabled || TEMP_CH != VBAT_CH)
        disableChannel(TEMP_CH);
}

void InternalADC::enableVbat(SampleTime sampleTime)
{
    vbatEnabled = true;
    enableChannel(VBAT_CH, sampleTime);
}

void InternalADC::disableVbat()
{
    vbatEnabled = false;
    if (!tempEnabled || TEMP_CH != VBAT_CH)
        disableChannel(VBAT_CH);
}

ADCData InternalADC::getVoltage(Channel channel)
{
    ADCData data;
    data.voltageTimestamp = getLastSample().timestamp;
    data.voltage          = getLastSample().voltage[channel];
    data.channelId        = channel;
    return data;
}

TemperatureData InternalADC::getTemperature()
{
    TemperatureData data;
    data.temperatureTimestamp = getLastSample().temperature;
    data.temperature          = getLastSample().temperature;
    return data;
}

ADCData InternalADC::getVbatVoltage()
{
    ADCData data;
    data.voltageTimestamp = getLastSample().timestamp;
    data.voltage          = getLastSample().vBat;
    data.channelId        = VBAT_CH;
    return data;
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

uint16_t InternalADC::readChannel(Channel channel)
{
    // Assuming that ADC_SQR1_L remains 0 (1 conversion)

    // Select channel
    adc->SQR3 = channel;

    // Start conversion
    adc->CR2 |= ADC_CR2_SWSTART;

    while (!(adc->SR & ADC_SR_EOC))
        ;

    return static_cast<uint16_t>(adc->DR);
}

#ifndef INTERNAL_ADC_WITHOUT_CALIBRATION
void InternalADC::loadCalibrationValues()
{
    calPt1Voltage = static_cast<float>(*CAL_PT1_VALUE);
    calPt1Voltage *= CAL_V_DDA / ADC_RESOLUTION;

    calPt2Voltage = static_cast<float>(*CAL_PT2_VALUE);
    calPt2Voltage *= CAL_V_DDA / ADC_RESOLUTION;

    calSlope = calPt2Voltage - calPt1Voltage;
    calSlope /= CAL_PT2_TEMP - CAL_PT1_TEMP;
}
#endif

}  // namespace Boardcore
