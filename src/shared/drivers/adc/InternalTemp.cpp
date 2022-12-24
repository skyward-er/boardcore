/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Giulia Ghirardini
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

#include "InternalTemp.h"

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
#warning This microcontroller does not have a calibrated temperature sensor or is not currently supported by this driver
#define WITHOUT_CALIBRATION
#endif

namespace Boardcore
{

InternalTemp::InternalTemp(InternalADC::SampleTime sampleTime,
                           const float supplyVoltage)
    : adc(ADC1, supplyVoltage), sampleTime(sampleTime)
{
}

bool InternalTemp::init()
{
    bool result = adc.init();

#if defined(STM32F407xx) || defined(STM32F205xx)
    adc.addRegularChannel(InternalADC::CH16);
#elif defined(STM32F429xx) || defined(STM32F767xx) || defined(STM32F769xx)
    adc.addRegularChannel(InternalADC::CH18);
#endif

    ADC->CCR &= ~ADC_CCR_VBATE;
    ADC->CCR |= ADC_CCR_TSVREFE;

    return result;
}

bool InternalTemp::selfTest() { return adc.selfTest(); }

InternalTempData InternalTemp::sampleImpl()
{
#if defined(STM32F407xx) || defined(STM32F205xx)
    auto adcData = adc.readChannel(InternalADC::CH16, sampleTime);
#elif defined(STM32F429xx) || defined(STM32F767xx) || defined(STM32F769xx)
    auto adcData = adc.readChannel(InternalADC::CH18, sampleTime);
#endif

    InternalTempData data;
    data.temperatureTimestamp = adcData.voltageTimestamp;

#ifdef WITHOUT_CALIBRATION
    // Default conversion
    data.temperature = ((adcData.voltage - 0.76) / 0.0025) + 25;
#else
    // Factory calibration
    float voltage30  = static_cast<float>(*TEMP30_CAL_VALUE) * 3.3 / 4095;
    float voltage110 = static_cast<float>(*TEMP110_CAL_VALUE) * 3.3 / 4095;
    float slope      = (voltage110 - voltage30) / (TEMP110 - TEMP30);
    data.temperature = ((adcData.voltage - voltage30) / slope) + TEMP30;
#endif

    return data;
}

}  // namespace Boardcore
