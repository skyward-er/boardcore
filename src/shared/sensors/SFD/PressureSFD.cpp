/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Federico Lolli
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

#include "PressureSFD.h"

#include <sensors/Sensor.h>
#include <sensors/SensorData.h>

#include <Eigen/Core>

namespace Boardcore
{

PressureSFD::PressureSFD(const SFDConfig& config)
    : sfdAscent(config.sfdAscentConfig), sfdDescent(config.sfdDescentConfig),
      currentMode(SFDMode::UNSET), k(0), weights({0}),
      lowPassFilter(config.lowPassConfig), medianFilter(), pressSamplers({0}),
      disabledSensors({0}), sampleWindow()
{
    // fill the weights array
    std::fill(weights.begin(), weights.end(), 1.0);
}

void PressureSFD::setPressureSample(std::function<PressureData()> sample,
                                    int index)
{
    pressSamplers[index] = sample;
}

void PressureSFD::setMode(SFDMode mode)
{
    currentMode = mode;
    // update k (step size)
    switch (mode)
    {
        case SFDMode::ACTIVE_ASCENT:
            k = K_ACTIVE_ASCENT;
            break;

        case SFDMode::PASSIVE_ASCENT:
            k = K_PASSIVE_ASCENT;
            break;

        case SFDMode::APOGEE_PROXIMITY:
            k = K_APOGEE_PROXIMITY;
            break;

        case SFDMode::DESCENT:
            k = K_DESCENT;
            break;

        case SFDMode::UNSET:
            break;
    }
}

PressureSFDData PressureSFD::sampleImpl()
{
    PressureSFDData data;
    PressureData sample;
    std::array<float, MASKED_SENSORS> samples;

    // get the samples from the sampler and get the latest timestamp
    for (int i = 0; i < MASKED_SENSORS; i++)
    {
        sample = pressSamplers[i]();

        samples[i] = sample.pressure;

        // update the timestamp based on the last sample
        if (i == 0)
            data.pressureTimestamp = sample.pressureTimestamp;
        else
            data.pressureTimestamp =
                std::max(data.pressureTimestamp, sample.pressureTimestamp);
    }

    // update the sliding window
    sampleWindow.push(samples);

    // if the windows is full and the mode is set we proceed by using the sfd
    // classification to exclude nodes otherwise we leave all the nodes enabled
    if (sampleWindow.isFull() && currentMode != SFDMode::UNSET)
        setDisabledSensors();

    stepWeights();

    float processed_value;

    processed_value = getWeightedAverage(samples);
    processed_value = medianFilter.filter(processed_value);
    processed_value = lowPassFilter.filter(processed_value);

    data.pressure = processed_value;

    return data;
}

void PressureSFD::setDisabledSensors()
{
    // get the samples for each sensor from the sliding window (transpose)
    SFDVectorIn sensor_window;
    std::array<SFDVectorIn, MASKED_SENSORS> sensor_samples;
    const auto& full_samples = sampleWindow.all();

    // fill eigen vectors with samples stored in window
    for (int i = 0; i < MASKED_SENSORS; i++)
    {
        sensor_window = SFDVectorIn::Zero();
        for (size_t j = 0; j < WIN_LEN; j++)
        {
            sensor_window(j) = full_samples[j][i];
        }
        sensor_samples[i] = sensor_window;
    }

    unsigned int skipped = 0;
    switch (currentMode)
    {
        case SFDMode::ACTIVE_ASCENT:
        case SFDMode::PASSIVE_ASCENT:
        case SFDMode::APOGEE_PROXIMITY:
            for (int i = 0; i < MASKED_SENSORS; i++)
            {
                if (disabledSensors[i])
                {
                    skipped++;
                    continue;
                }
                if (sfdAscent.classify(sensor_samples[i]))
                {
                    disabledSensors[i] = true;
                    skipped++;
                }
            }
            break;

        case SFDMode::DESCENT:
            for (int i = 0; i < MASKED_SENSORS; i++)
            {
                if (disabledSensors[i])
                {
                    skipped++;
                    continue;
                }
                if (sfdDescent.classify(sensor_samples[i]))
                {
                    disabledSensors[i] = true;
                    skipped++;
                }
            }
            break;

        case SFDMode::UNSET:
            break;
    }

    // reset all sensors if all of them are faulty
    if (skipped == MASKED_SENSORS)
    {
        for (int i = 0; i < MASKED_SENSORS; i++)
        {
            disabledSensors[i] = false;
        }
    }
}

void PressureSFD::stepWeights()
{
    for (int i = 0; i < MASKED_SENSORS; i++)
    {
        if (disabledSensors[i])
            weights[i] = std::max(weights[i] - k, MIN_WEIGHT);
        else
            weights[i] = std::min(weights[i] + k, MAX_WEIGHT);
    }
}

float PressureSFD::getWeightedAverage(std::array<float, MASKED_SENSORS> samples)
{
    float weighted_sum = 0, weights_sum = 0;
    for (int i = 0; i < MASKED_SENSORS; i++)
    {
        weighted_sum += samples[i] * weights[i];
        weights_sum += weights[i];
    }
    return weighted_sum / weights_sum;
}

}  // namespace Boardcore
