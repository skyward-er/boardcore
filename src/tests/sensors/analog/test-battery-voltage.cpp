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
#include <miosix.h>
#include <sensors/analog/battery/BatteryVoltageSensor.h>
#include <utils/Debug.h>

using namespace Boardcore;
using namespace miosix;

GpioPin batteryPin               = GpioPin(GPIOA_BASE, 3);
InternalADC::Channel ADC_CHANNEL = InternalADC::Channel::CH3;
ADC_TypeDef& ADCx                = *ADC3;
InternalADC adc(ADCx);

void initBoard()
{
    {
        miosix::FastInterruptDisableLock dLock;

        batteryPin.mode(miosix::Mode::INPUT_ANALOG);

        // Set pins PA0 PA1 PA2 PA3 as analog input
        // RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        // GPIOA->MODER = 0xFF;

        // Enable ADC3 clock
        RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;  // <- CHANGE THIS!

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

    BatteryVoltageSensor batterySensor(getVoltageFunction, 0.2063);

    if (!adc.init() || !adc.selfTest())
    {
        printf("ERROR : %d\n", adc.getLastError());
    }
    else
    {
        batterySensor.init();

        // Read samples with sample()
        while (1)
        {
            adc.sample();

            miosix::Thread::sleep(100);

            batterySensor.sample();

            BatteryVoltageSensorData batData = batterySensor.getLastSample();
            printf("%llu %u %f %f \n", batData.voltageTimestamp,
                   batData.channelId, batData.voltage, batData.batVoltage);

            miosix::Thread::sleep(100);
        }
    }
}
