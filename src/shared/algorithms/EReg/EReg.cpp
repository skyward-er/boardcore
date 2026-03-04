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

#include <math.h>

namespace Boardcore
{

EReg::EReg(const ERegPIDConfig& pidConfig, const ERegValveInfo& valveInfo,
           float targetPressure)
    : pidConfig(pidConfig), valveInfo(valveInfo), targetPressure(targetPressure)
{
}

void EReg::setReferencePoint(float newTargetPressure)
{
    targetPressure = newTargetPressure;
    resetState();
}

void EReg::changePIDConfig(const ERegPIDConfig& newConfig)
{
    pidConfig = newConfig;
    resetState();
}

void EReg::setIntegralContribution(float newIntegral) { i = newIntegral; };

void EReg::setInput(float downstreamPressure, float upstreamPressure)
{
    downstreamPressureSample = downstreamPressure;
    upstreamPressureSample   = upstreamPressure;
}

float EReg::getOutput() { return nextServoPosition; }

void EReg::step()
{
    float error = targetPressure - downstreamPressureSample;

    float d = (error - lastError) / pidConfig.Ts * pidConfig.KD;

    if (!saturation)
        i = i + pidConfig.KI * pidConfig.Ts * error;

    float u =
        (pidConfig.KP * error + i + d) /
        std::sqrt(std::abs(upstreamPressureSample - downstreamPressureSample));

    lastError         = error;
    nextServoPosition = antiWindUp(u);
}

float EReg::antiWindUp(float PIDCommand)
{
    float servoCommand;

    if (PIDCommand <= 0.0f)
    {
        saturation   = true;
        servoCommand = valveInfo.minServoPosition;
    }
    else if (PIDCommand >= 1.0f)
    {
        saturation   = true;
        servoCommand = 1.0f;
    }
    else
    {
        saturation   = false;
        servoCommand = convertCvToServoCommand(PIDCommand);
    }

    return servoCommand;
}

float EReg::convertCvToServoCommand(float PIDCommand)
{
    float servoCommand;

    // Rescale the cv to the range [0, maxCv]
    PIDCommand *= valveInfo.maxCv;

    // clang-format off
        // Calculate the angle of the valve from the cv
        servoCommand = PIDCommand * (PIDCommand * (PIDCommand * (PIDCommand * (valveInfo.polyValveCoeff[0] *
        PIDCommand + valveInfo.polyValveCoeff[1]) + valveInfo.polyValveCoeff[2]) + valveInfo.polyValveCoeff[3])
        + valveInfo.polyValveCoeff[4]) + valveInfo.polyValveCoeff[5];

        // Rescale to the range [minAngle, 90]
        servoCommand = servoCommand * (90 - valveInfo.minValveAngle) + valveInfo.minValveAngle; 

        // Calculate the position of the servo from the angle of the valve
        servoCommand = servoCommand * (servoCommand * (servoCommand * (servoCommand * (valveInfo.polyServoCoeff[0] * 
        servoCommand + valveInfo.polyServoCoeff[1]) + valveInfo.polyServoCoeff[2]) + valveInfo.polyServoCoeff[3])
        + valveInfo.polyServoCoeff[4]) + valveInfo.polyServoCoeff[5];
    // clang-format on

    return servoCommand;
}

void EReg::resetState()
{
    i          = 0.0f;
    lastError  = 0.0f;
    saturation = false;
}

}  // namespace Boardcore
