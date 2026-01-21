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

        enum class Activation
        {
            BACKWARDS   = -1,
            STOP        = 0,
            FORWARD     = 1
            
        };
        /**
         * @brief Constructor for the SchmittTrigger class.
         */
        SchmittTrigger();

        /**
         * @brief Constructor for the SchmittTrigger class.
         *
         * @param thresholds The thresholds array for the trigger.
         */
        SchmittTrigger(uint8_t thresholds[2]);

        /**
         * @brief Initializes the SchmittTrigger object.
         *
         * @return true if initialization is successful, false otherwise.
         */
        bool init() override;

        /**
         * @brief Sets the thresholds for the Schmitt Trigger.
         *
         * @param thresholds The thresholds array to be set, first element is the lower threshold and second is the upper threshold.
         */
        void setThresholds(uint8_t newThresholds[2]);

        /**
         * @brief Sets the state value for the Schmitt Trigger.
         *
         * @param state The state value to be evaluated.
         */
        void setState(uint16_t newState);

        /**
         * @brief Sets the reference value for the Schmitt Trigger.
         *
         * @param reference The reference value to be used in the evaluation.
         */
        void setReference(uint16_t newReference);

        /**
         * @brief Returns the current activation state of the Schmitt Trigger.
         *
         * @return The current activation state.
         */
        Activation getActivation();

    protected:
        uint16_t    state;
        uint16_t    reference;
        uint8_t     thresholds[2];
        Activation  activation;

        bool thresholdsSet = false;

        PrintLogger logger = Logging::getLogger("SchmittTrigger");

        miosix::FastMutex schmittMutex;

        void step() override;
    }
}