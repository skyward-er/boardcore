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

#ifdef _ARCH_CORTEXM4_STM32F4
#define TEMP30_CAL_VALUE ((uint16_t*)((uint32_t)0x1FFF7A2C))
#define TEMP110_CAL_VALUE ((uint16_t*)((uint32_t)0x1FFF7A2E))
#define TEMP30 30
#define TEMP110 110
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

#if defined(_BOARD_STM32F4DISCOVERY) || defined(_ARCH_CORTEXM3_STM32F2)
    adc.addRegularChannel(InternalADC::CH16);
#else
    adc.addRegularChannel(InternalADC::CH18);
#endif

    ADC->CCR &= ~ADC_CCR_VBATE;
    ADC->CCR |= ADC_CCR_TSVREFE;

    return result;
}

bool InternalTemp::selfTest() { return adc.selfTest(); }

InternalTempData InternalTemp::sampleImpl()
{
#if defined(_BOARD_STM32F4DISCOVERY) || defined(_ARCH_CORTEXM3_STM32F2)
    auto adcData = adc.readChannel(InternalADC::CH16, sampleTime);
#else
    auto adcData = adc.readChannel(InternalADC::CH18, sampleTime);
#endif

    InternalTempData data;
    data.temperatureTimestamp = adcData.voltageTimestamp;

#ifdef _ARCH_CORTEXM3_STM32F2
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
