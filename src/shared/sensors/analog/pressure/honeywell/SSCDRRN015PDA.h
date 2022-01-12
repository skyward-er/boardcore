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

#include <math/Stats.h>

#include "HoneywellPressureSensor.h"
#include "SSCDRRN015PDAData.h"

namespace Boardcore
{

/**
 * @brief Differential pressure sensor with a ±103kPa range (±15psi)
 */
class SSCDRRN015PDA final : public HoneywellPressureSensor<SSCDRRN015PDAData>
{
public:
    SSCDRRN015PDA(std::function<ADCData()> getSensorVoltage_,
                  const float V_SUPPLY_                 = 5.0,
                  const unsigned int num_calib_samples_ = 200)
        : HoneywellPressureSensor(getSensorVoltage_, V_SUPPLY_, 103421.3594,
                                  -103421.3594),
          offset(0.0), num_calib_samples(num_calib_samples_)
    {
    }

    SSCDRRN015PDAData sampleImpl() override
    {
        lastSample = HoneywellPressureSensor<SSCDRRN015PDAData>::sampleImpl();

        if (calibrating)
        {
            press_stats.add(lastSample.pressure);

            if (press_stats.getStats().nSamples >= num_calib_samples)
            {
                calibrating = false;
                offset      = press_stats.getStats().mean;

                TRACE("Differential barometer offset : %.2f \n", offset);
            }
        }

        lastSample.pressure = lastSample.pressure - offset;

        return lastSample;
    }

    void calibrate()
    {
        press_stats.reset();
        offset      = 0.0f;
        calibrating = true;
    }

    bool isCalibrating() { return calibrating; }

private:
    bool calibrating = false;
    float offset;
    Stats press_stats;
    unsigned int num_calib_samples;
};

}  // namespace Boardcore
