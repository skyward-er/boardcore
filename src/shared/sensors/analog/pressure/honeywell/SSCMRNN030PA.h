/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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

#include "HoneywellPressureSensor.h"
#include "SSCMRNN030PAData.h"

namespace Boardcore
{

/**
 * @brief Absolute pressure sensor with a 0-206kPa range (0-30psi)
 */
class SSCMRNN030PA : public HoneywellPressureSensor<SSCMRNN030PAData>
{
public:
    SSCMRNN030PA(std::function<ADCData()> getSensorVoltage,
                 const float supplyVoltage = 5.0)
        : HoneywellPressureSensor(getSensorVoltage, supplyVoltage, 206842.7188)
    {
    }
};

}  // namespace Boardcore
