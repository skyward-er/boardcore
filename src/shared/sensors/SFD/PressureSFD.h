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

#pragma once

#include <algorithms/Filters/AdjustableMedFilter.h>
#include <algorithms/Filters/LowPass.h>
#include <algorithms/SFD/SFDAscent.h>
#include <algorithms/SFD/SFDCommon.h>
#include <algorithms/SFD/SFDDescent.h>
#include <sensors/Sensor.h>
#include <sensors/SensorData.h>
#include <utils/SlidingWindow.h>

#include "PressureSFDData.h"

namespace Boardcore
{

enum class SFDMode
{
    UNSET,
    ACTIVE_ASCENT,
    PASSIVE_ASCENT,
    APOGEE_PROXIMITY,
    DESCENT
};

class PressureSFD : public Sensor<PressureSFDData>
{
public:
    static constexpr int MASKED_SENSORS = 2;
    static constexpr int WIN_LEN        = LEN_CHUNK;

    // TODO: set these values
    static constexpr int MED_FILTER_WIN_LEN   = 25;
    static constexpr float K_ACTIVE_ASCENT    = 0;
    static constexpr float K_PASSIVE_ASCENT   = 0;
    static constexpr float K_APOGEE_PROXIMITY = 0;
    static constexpr float K_DESCENT          = 0;
    static constexpr float MAX_WEIGHT         = 1;
    static constexpr float MIN_WEIGHT         = 0;

    using SVMAscVec  = Eigen::Vector<float, SFDAscent::NUM_FEATURES>;
    using SVMDescVec = Eigen::Vector<float, SFDDescent::NUM_FEATURES>;

    struct SFDConfig
    {
        SFDAscent::SFDAscentConfig sfdAscentConfig;
        SFDDescent::SFDDescentConfig sfdDescentConfig;
        LowPass::LowPassConfig lowPassConfig;
    };

    explicit PressureSFD(const SFDConfig& config);

    bool init() override { return true; }
    bool selfTest() override { return true; }

    void setPressureSample(std::function<PressureData()> sample, int index);

    void setMode(SFDMode mode);

private:
    void setDisabledSensors();
    void stepWeights();
    float getWeightedAverage(std::array<float, MASKED_SENSORS> samples);
    PressureSFDData sampleImpl() override;

    SFDAscent sfdAscent;
    SFDDescent sfdDescent;
    SFDMode currentMode;

    float k;
    std::array<float, MASKED_SENSORS> weights;

    LowPass lowPassFilter;
    AdjustableMedFilter<MED_FILTER_WIN_LEN> medianFilter;

    std::array<std::function<PressureData()>, MASKED_SENSORS> pressSamplers;
    std::array<bool, MASKED_SENSORS> disabledSensors;

    SlidingWindow<std::array<float, MASKED_SENSORS>, WIN_LEN> sampleWindow;
};

}  // namespace Boardcore
