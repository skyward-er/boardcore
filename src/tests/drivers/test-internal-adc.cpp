/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include <drivers/adc/InternalADC/InternalADC.h>
#include <miosix.h>

#include "TimestampTimer.h"

ADC_TypeDef& ADCx = *ADC3;

int main()
{
    // Set the clock divider for the analog circuitry (/8)
    ADC->CCR |= ADC_CCR_ADCPRE_0 | ADC_CCR_ADCPRE_1;
    // In this case I've set the maximum value, check the datasheet for the
    // maximum frequency the analog circuitry supports and compare it with the
    // parent clock

    TimestampTimer::enableTimestampTimer();

    InternalADC adc(ADCx, 3.3);
    adc.enableChannel(InternalADC::CH4);  // PF6
    adc.enableChannel(InternalADC::CH5);  // PF7
    adc.enableChannel(InternalADC::CH6);  // PF8
    adc.init();

    printf("Configuration completed\n");

    while (1)
    {
        adc.sample();

        printf("CH4:%1.3f\tCH5:%1.3f\tCH6:%1.3f\n",
               adc.getVoltage(InternalADC::CH4).voltage,
               adc.getVoltage(InternalADC::CH5).voltage,
               adc.getVoltage(InternalADC::CH6).voltage);

        miosix::delayMs(1000);
    }
}