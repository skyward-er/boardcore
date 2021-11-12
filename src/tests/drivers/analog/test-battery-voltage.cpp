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

#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>

#include "Debug.h"
#include "drivers/adc/InternalADC/InternalADC.h"
#include "sensors/analog/battery/BatteryVoltageSensor.h"

using namespace miosix;

GpioPin battery_pin              = GpioPin(GPIOA_BASE, 3);
InternalADC::Channel ADC_CHANNEL = InternalADC::Channel::CH3;
ADC_TypeDef& ADCx                = *ADC3;
InternalADC adc(ADCx);

void initBoard()
{
    {
        miosix::FastInterruptDisableLock dLock;

        battery_pin.mode(miosix::Mode::INPUT_ANALOG);

        // Set pins PA0 PA1 PA2 PA3 as analog input
        // RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        // GPIOA->MODER = 0xFF;

        // Enable ADC3 clock
        RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;  // <- CHANGE THIS!

        // Set the clock divider for the analog circuitry (/8)
        ADC->CCR |= ADC_CCR_ADCPRE_0 | ADC_CCR_ADCPRE_1;
    }

    TimestampTimer::enableTimestampTimer();
}

int main()
{
    // Enable SPI clock and set gpios
    initBoard();

    adc.enableChannel(ADC_CHANNEL);

    printf("Configuration completed\n");

    std::function<ADCData()> get_voltage_function = []()
    { return adc.getVoltage(ADC_CHANNEL); };
    // std::bind(&InternalADC::getVoltage, adc, ADC_CHANNEL);

    BatteryVoltageSensor battery_sensor(get_voltage_function, 0.2063);

    if (!adc.init() || !adc.selfTest())
    {
        printf("ERROR : %d\n", adc.getLastError());
    }
    else
    {
        battery_sensor.init();

        // Read samples with sample()
        while (1)
        {
            adc.sample();

            miosix::Thread::sleep(100);

            battery_sensor.sample();

            BatteryVoltageSensorData bat_data = battery_sensor.getLastSample();
            printf("%llu %u %f %f \n", bat_data.adc_timestamp,
                   bat_data.channel_id, bat_data.voltage, bat_data.bat_voltage);

            miosix::Thread::sleep(100);
        }
    }
}