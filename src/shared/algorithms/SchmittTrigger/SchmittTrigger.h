/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Leonardo Montecchi
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
#include <diagnostic/PrintLogger.h>
#include <logger/Logger.h>

namespace Boardcore
{
/**
 * @brief Schmitt Trigger algorithm class.
 *
 * The Schmitt Trigger class takes 2 thresholds (low and high), a target and
 * a state and as an output returns an activation value that is:
 *     STOP    if the state is between the thresholds of the target;
 *     HIGH    if the state is below the low threshold of the target;
 *     LOW     if the state is above the high threshold of the target.
 */
class SchmittTrigger : public Algorithm
{
public:
    enum class Activation
    {
        LOW,
        STOP,
        HIGH

    };

    /**
     * @brief Constructor for the SchmittTrigger class.
     */
    SchmittTrigger(float thresholdLow, float thresholdHigh);

    /**
     * @brief Initializes the SchmittTrigger object.
     *
     * @return Always true.
     */
    bool init() override;

    /**
     * @brief Setter function for the thresholds for the Schmitt Trigger.
     */
    void setThresholds(float thresholdLow, float thresholdHigh);

    /**
     * @brief Setter function for the state value for the Schmitt Trigger.
     */
    void setCurrentState(float state);

    /**
     * @brief Setter function for the target value for the Schmitt Trigger.
     */
    void setTargetState(float target);

    /**
     * @brief Getter function for the current activation state of the Schmitt
     * Trigger.
     *
     * @return The current activation state: LOW, STOP, or HIGH.
     */
    Activation getOutput();

protected:
    float state  = 0.0f;
    float target = 0.0f;
    float thresholdLow;
    float thresholdHigh;

    Activation activation = Activation::STOP;

    miosix::FastMutex schmittMutex;

    void step() override;
};
}  // namespace Boardcore
