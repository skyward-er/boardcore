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

#pragma once

#include "../AnalogPressureSensor.h"
#include "MPXHZ6130AData.h"

/**
 * @brief Driver for NXP's MPXHZ6130A pressure sensor
 */
class MPXHZ6130A final : public AnalogPressureSensor<MPXHZ6130AData>
{
public:
    MPXHZ6130A(std::function<ADCData()> getSensorVoltage_,
               const float V_SUPPLY_                 = 5.0,
               const unsigned int num_calib_samples_ = 200,
               const float moving_avg_coeff_         = 1.0,
               const float ref_press_                = 101325.0f)
        : AnalogPressureSensor(getSensorVoltage_, V_SUPPLY_, 130000),
          offset(0.0), num_calib_samples(num_calib_samples_),
          moving_avg_coeff(moving_avg_coeff_), ref_press(ref_press_)
    {
    }

    MPXHZ6130AData sampleImpl()
    {
        last_sample = AnalogPressureSensor<MPXHZ6130AData>::sampleImpl();

        if (calibrating)
        {
            press_stats.add(last_sample.press);

            if (press_stats.getStats().nSamples >= num_calib_samples)
            {
                calibrating = false;
                offset      = ref_press - press_stats.getStats().mean;

                TRACE("MPXHZ6130A barometer offset : %.2f \n", offset);
            }
        }

        last_sample.press = last_sample.press + offset;

        last_sample.press = movingAverage(last_sample.press);

        return last_sample;
    }

    void setReferencePressure(float p) { ref_press = p; }

    void calibrate()
    {
        press_stats.reset();
        offset      = 0.0f;
        calibrating = true;
    }

    bool isCalibrating() { return calibrating; }

private:
    float voltageToPressure(float voltage) override
    {
        return (((voltage / V_SUPPLY) + CONST_B) / CONST_A) * 1000;
    }

    float movingAverage(float new_value)
    {
        accumulator = (moving_avg_coeff * new_value) +
                      (1.0 - moving_avg_coeff) * accumulator;
        return accumulator;
    }

    // Constants from datasheet
    static constexpr float CONST_A = 0.007826;
    static constexpr float CONST_B = 0.07739;

    bool calibrating = false;
    float offset;
    Stats press_stats;
    unsigned int num_calib_samples;

    // moving average
    const float moving_avg_coeff;
    float accumulator = 0.0;

    float ref_press;
};