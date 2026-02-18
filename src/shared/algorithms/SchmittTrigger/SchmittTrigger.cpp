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

#include "SchmittTrigger.h"

#include <diagnostic/PrintLogger.h>
#include <logger/Logger.h>

using namespace miosix;

namespace Boardcore
{
SchmittTrigger::SchmittTrigger(float thresholdLow, float thresholdHigh)
{
    setThresholds(thresholdLow, thresholdHigh);
}

bool SchmittTrigger::init()
{
    Lock<FastMutex> lock(schmittMutex);
    return true;
}

void SchmittTrigger::setThresholds(float thresholdLow, float thresholdHigh)
{
    Lock<FastMutex> lock(schmittMutex);
    this->thresholdLow  = thresholdLow;
    this->thresholdHigh = thresholdHigh;
}

void SchmittTrigger::setState(float state)
{
    Lock<FastMutex> lock(schmittMutex);
    this->state = state;
}

void SchmittTrigger::setReference(float reference)
{
    Lock<FastMutex> lock(schmittMutex);
    this->reference = reference;
}

SchmittTrigger::Activation SchmittTrigger::getActivation()
{
    Lock<FastMutex> lock(schmittMutex);
    return activation;
}

void SchmittTrigger::step()
{
    Lock<FastMutex> lock(schmittMutex);

    if (state <= reference - thresholdLow)
        activation = Activation::HIGH;
    else if (state >= reference + thresholdHigh)
        activation = Activation::LOW;
    else
        activation = Activation::STOP;
}

}  // namespace Boardcore
