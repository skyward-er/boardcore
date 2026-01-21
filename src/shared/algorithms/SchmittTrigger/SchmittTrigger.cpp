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

#include "SchmittTrigger.h"
#include <diagnostic/PrintLogger.h>
#include <logger/Logger.h>

using namespace miosix;

namespace Boardcore
{
    SchmittTrigger::SchmittTrigger()
    {
        Lock<FastMutex> lock(schmittMutex);
        activation    = Activation::STOP; //Vedere se ha senso inizializzare a STOP
    }
 
    SchmittTrigger::SchmittTrigger(uint8_t inputThresholds[2])
    {
        Lock<FastMutex> lock(schmittMutex);
        thresholds[0] = inputThresholds[0];
        thresholds[1] = inputThresholds[1];
        activation    = Activation::STOP; //Vedere se ha senso inizializzare a STOP
        thresholdsSet = true;
    }

    bool SchmittTrigger::init()
    {
        Lock<FastMutex> lock(schmittMutex);
        if(!thresholdsSet)
        {
            LOG_ERR(logger, "Thresholds not set\n");
            return false;
        }
        return true;
    }

    void SchmittTrigger::setThresholds(uint8_t newThresholds[2])
    {
        Lock<FastMutex> lock(schmittMutex);
        thresholds[0] = newThresholds[0];
        thresholds[1] = newThresholds[1];
        thresholdsSet = true;
    }

    void SchmittTrigger::setState(uint16_t newState)
    {
        Lock<FastMutex> lock(schmittMutex);
        state = newState;
    }

    void SchmittTrigger::setReference(uint16_t newReference)
    {
        Lock<FastMutex> lock(schmittMutex);
        reference = newReference;
    }

    SchmittTrigger::Activation SchmittTrigger::getActivation()
    {
        Lock<FastMutex> lock(schmittMutex);
        return activation;
    }

    void SchmittTrigger::step()
    {
        Lock<FastMutex> lock(schmittMutex);

        if(!thresholdsSet)
        {
            LOG_ERR(logger, "Thresholds not set\n");
            return;
        }

        //TODO: controllare bene logica in caso di overflow

        if (state <= reference - thresholds[0])
        {
            activation = Activation::FORWARD;
        }
        else if (state >= reference + thresholds[1])
        {
            activation = Activation::BACKWARDS;
        }
        else
        {
            activation = Activation::STOP;
        }
    }

}