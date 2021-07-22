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

#pragma once

#include <sensors/Sensor.h>

#include <functional>

/**
 * @brief Common class for battery voltage sensors
 *
 * It needs the conversion ratio, in order to convert the raw voltage to
 * battery voltage.
 */
class BatteryVoltageSensor : public Sensor<BatteryVoltageData>
{
public:
    static constexpr int MOVING_AVAERAGE_N = 20;

    BatteryVoltageSensor(std::function<ADCData()> getADCVoltage_,
                         float conversionCoeff_)
        : getADCVoltage(getADCVoltage_), conversionCoeff(conversionCoeff_)
    {
        last_sample.bat_voltage = 0;
    }

    bool init() override { return true; };

    bool selfTest() override { return true; };

    ///< Converts the adc voltage value to battery voltage
    BatteryVoltageData sampleImpl() override
    {
        ADCData adc_data = getADCVoltage();

        if (last_sample.bat_voltage == 0)
        {
            last_sample.bat_voltage = adcToBatteryVoltage(adc_data.voltage);
        }

        BatteryVoltageData bat_data;
        bat_data.adc_timestamp = adc_data.adc_timestamp;
        bat_data.channel_id    = adc_data.channel_id;
        bat_data.voltage       = adc_data.voltage;

        // Moving average
        bat_data.bat_voltage =
            last_sample.bat_voltage * MOVING_AVAERAGE_COMP_COEFF;
        bat_data.bat_voltage +=
            adcToBatteryVoltage(adc_data.voltage) * MOVING_AVAERAGE_COEFF;

        return bat_data;
    }

private:
    ///< Conversion function from adc volts to battery volts
    float adcToBatteryVoltage(float voltage)
    {
        return voltage * conversionCoeff;
    }

    ///< Function that returns the adc voltage
    std::function<ADCData()> getADCVoltage;

    float conversionCoeff;

    static constexpr float MOVING_AVAERAGE_COEFF = 1 / (float)MOVING_AVAERAGE_N;
    static constexpr float MOVING_AVAERAGE_COMP_COEFF =
        1 - MOVING_AVAERAGE_COEFF;
};