/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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
#include <interfaces-impl/gpio_impl.h>
#include <miosix.h>
#include <sensors/analog/CurrentSensor.h>
#include <utils/Debug.h>

using namespace miosix;
using namespace Boardcore;

GpioPin batteryPin               = GpioPin(GPIOA_BASE, 3);
InternalADC::Channel ADC_CHANNEL = InternalADC::Channel::CH3;
InternalADC adc(ADC3);

void initBoard()
{
    {
        miosix::FastInterruptDisableLock dLock;

        batteryPin.mode(miosix::Mode::INPUT_ANALOG);

        // Set the clock divider for the analog circuitry (/8)
        ADC->CCR |= ADC_CCR_ADCPRE_0 | ADC_CCR_ADCPRE_1;
    }
}

int main()
{
    // Enable SPI clock and set gpios
    initBoard();

    adc.enableChannel(ADC_CHANNEL);

    printf("Configuration completed\n");

    std::function<ADCData()> getVoltageFunction = []()
    { return adc.getVoltage(ADC_CHANNEL); };
    // std::bind(&InternalADC::getVoltage, adc, ADC_CHANNEL);
    // example transfer function
    std::function<float(float)> adcToCurrent = [](float adcIn)
    { return (adcIn - 107.0f) * 32.4f; };
    CurrentSensor currentSensor(getVoltageFunction, adcToCurrent);

    if (!adc.init() || !adc.selfTest())
    {
        printf("ERROR : %d\n", adc.getLastError());
    }
    else
    {
        currentSensor.init();

        // Read samples with sample()
        while (1)
        {
            adc.sample();

            miosix::Thread::sleep(500);

            currentSensor.sample();

            CurrentSensorData currentData = currentSensor.getLastSample();
            printf("%llu %u %f %f \n", currentData.voltageTimestamp,
                   currentData.channelId, currentData.voltage,
                   currentData.current);

            miosix::Thread::sleep(100);
        }
    }
}
