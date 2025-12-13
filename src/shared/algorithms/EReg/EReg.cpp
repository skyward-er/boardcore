/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Pietro Bortolus
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

#include "EReg.h"

namespace Boardcore
{

EReg::EReg(const ERegPIDConfig& config, float targetPressure,
           std::function<float()> getPressure,
           std::function<void(float)> setValvePosition)
    : config(config), targetPressure(targetPressure),
      getPressure(std::move(getPressure)),
      setValvePosition(std::move(setValvePosition))
{
}

void EReg::setReferencePoint(float newTargetPressure)
{
    targetPressure = newTargetPressure;
    resetState();
}

void EReg::changePIDConfig(const ERegPIDConfig& newConfig)
{
    config = newConfig;
    resetState();
}

void EReg::step()
{
    float sample = getPressure();

    float error = targetPressure - sample;

    d = (error - lastError) / config.Ts * config.KD;

    if (!saturation)
        i = i + config.KI * config.Ts * error;

    float u = config.KP * error + i + d;

    setValvePosition(antiWindUp(u, config.uMin, config.uMax));
}

float EReg::antiWindUp(float u, float uMin, float uMax)
{
    if (u < uMin)
    {
        u          = uMin;
        saturation = true;
    }
    else if (u > uMax)
    {
        u          = uMax;
        saturation = true;
    }
    else
    {
        saturation = false;
    }

    return u;
}

void EReg::resetState()
{
    i          = 0;
    d          = 0;
    lastError  = 0;
    saturation = false;
}

}  // namespace Boardcore
