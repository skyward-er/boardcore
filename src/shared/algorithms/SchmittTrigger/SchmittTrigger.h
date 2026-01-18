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
     * This class implements a Schmitt Trigger, which is a type of comparator circuit
     * with hysteresis. It is used to convert an analog input signal into a digital
     * output signal, providing noise immunity and preventing rapid switching.
     */
    class SchmittTrigger : public Algorithm
    {
    public:
        /**
         * @brief Constructor for the SchmittTrigger class.
         *
         * @param lowerThreshold The lower threshold value for the trigger.
         * @param upperThreshold The upper threshold value for the trigger.
         */
        SchmittTrigger(float lowerThreshold, float upperThreshold);

        /**
         * @brief Initializes the SchmittTrigger object.
         *
         * @return true if initialization is successful, false otherwise.
         */
        bool init() override;

        /**
         * @brief Sets the state value for the Schmitt Trigger.
         *
         * @param input The state value to be evaluated.
         */
        void setState(float input);

        /**
         * @brief Gets the current output state of the Schmitt Trigger.
         *
         * @return true if the output is high, false if it is low.
         */
        bool getOutput() const;
    protected:
        uint16_t    state;
        uint16_t    reference;
        uint8_t     thresholds[2];

        enum class Activation
        {
            STOP        = 0,
            FORWARD     = 1,
            BACKWARDS   = 2
        } currentState;
        
        bool outputState;

        void step() override;
    }
}