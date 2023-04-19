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

#include <drivers/adc/InternalADC.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;

GpioPin ch5(GPIOA_BASE, 5);
GpioPin ch6(GPIOA_BASE, 6);

int main()
{
    ch5.mode(Mode::INPUT_ANALOG);
    ch6.mode(Mode::INPUT_ANALOG);

    InternalADC adc(ADC1);
    adc.enableChannel(InternalADC::CH5);  // PA5
    adc.enableChannel(InternalADC::CH6);  // PA6
    adc.enableTemperature();
    adc.enableVbat();
    adc.init();

    while (true)
    {
        adc.sample();

        printf("CH5: %1.3f\tCH6: %1.3f\tTemp: %1.3f\tVbat: %1.3f\n",
               adc.getVoltage(InternalADC::CH5).voltage,
               adc.getVoltage(InternalADC::CH6).voltage,
               adc.getTemperature().temperature, adc.getVbatVoltage().voltage);

        delayMs(1000);
    }
}
