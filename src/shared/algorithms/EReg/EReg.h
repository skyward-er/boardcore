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

#pragma once

#include <algorithms/Algorithm.h>

#include <functional>
#include <limits>
#include <vector>

#include "ERegConfig.h"

#define DERIVATIVE_LIFO_SIZE 3

namespace Boardcore
{

class EReg : public Algorithm
{
public:
    EReg(const ERegPIDConfig& pidConfig, const ERegValveInfo& valveInfo,
         float targetPressure);

    bool init() { return true; }

    /**
     * @brief Change the reference point for the PID, this also resets the
     * state.
     */
    void setReferencePoint(float newTargetPressure);

    /**
     * @brief Change the config of the PID, this also resets the state.
     */
    void changePIDConfig(const ERegPIDConfig& newConfig);

    /**
     * @brief Change the value of the integral conntritution.
     */
    void setIntegralContribution(float newIntegral);

    /**
     * @brief Setter function to input pressures into the algorithm
     *
     * @param downstreamPressure The pressure downstream of the regulator
     * @param upstreamPressure The pressure upstream of the regulator
     */
    void setInput(float downstreamPressure, float upstreamPressure);

    /**
     * @brief Getter function to retrieve the output of the algorithm
     *
     * @return The next position the servo should move to
     */
    float getOutput();

protected:
    /**
     * @brief Update the PID internal state.
     * */
    void step() override;

private:
    /**
     * @brief Anti-windup mechanism.
     */
    float antiWindUp(float PIDCommand);

    float convertCvToServoCommand(float PIDCommand);

    /**
     * @brief Moving average filter for the derivative errors
     *
     * @param newValue The new value to replace the oldest one in the LIFO
     *
     * @return The average of the three values in the LIFO, values equal to 0.0f
     * are discarded
     */
    float derivativeAvgFilter(float newValue);

    /**
     * @brief Resets the PID internal state.
     */
    void resetState();

    ERegPIDConfig pidConfig;  // PID config.
    ERegValveInfo valveInfo;  // Information about the valve mechanism.

    float targetPressure;  // Reference point.

    float downstreamPressureSample = 0;
    float upstreamPressureSample   = 0;
    float nextServoPosition        = 0;

    float i         = 0;  // Integral contribution.
    float lastError = 0;  // Error at the previous step.

    float derivativeLifo[DERIVATIVE_LIFO_SIZE] = {
        0.0f, 0.0f, 0.0f};  // Circular buffer containing the last 3 values
    uint8_t derivativeErrorsIdx = 0;  // Index of the oldest value

    bool saturation = false;  // have we reached saturation?
};

}  // namespace Boardcore
