/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <sensors/LPS28DFW/LPS28DFWData.h>
#include <sensors/analog/pressure/honeywell/HSCMRNN015PAData.h>

namespace HILTest
{
struct SensorsCalibrationParameter
{
    uint64_t timestamp;
    float referencePressure;
    float offsetStatic1;
    float offsetStatic2;
    float offsetDeployment;

    SensorsCalibrationParameter(uint64_t timestamp, float referencePressure,
                                float offsetStatic1, float offsetStatic2,
                                float offsetDeployment)
        : timestamp(timestamp), referencePressure(referencePressure),
          offsetStatic1(offsetStatic1), offsetStatic2(offsetStatic2),
          offsetDeployment(offsetDeployment)
    {
    }

    SensorsCalibrationParameter() : SensorsCalibrationParameter(0, 0, 0, 0, 0)
    {
    }

    static std::string header()
    {
        return "timestamp,referencePressure,offsetStatic1,offsetStatic2,"
               "offsetDeployment\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << referencePressure << "," << offsetStatic1
           << "," << offsetStatic2 << "," << offsetDeployment << "\n";
    }
};
struct LPS28DFW_1Data : Boardcore::LPS28DFWData
{
    explicit LPS28DFW_1Data(const Boardcore::LPS28DFWData& data)
        : Boardcore::LPS28DFWData(data)
    {
    }

    LPS28DFW_1Data() {}

    static std::string header()
    {
        return "pressureTimestamp,pressure,temperatureTimestamp,temperature\n ";
    }

    void print(std::ostream& os) const
    {
        os << pressureTimestamp << "," << pressure << ","
           << temperatureTimestamp << "," << temperature << "\n";
    }
};

struct LPS28DFW_2Data : Boardcore::LPS28DFWData
{
    explicit LPS28DFW_2Data(const Boardcore::LPS28DFWData& data)
        : Boardcore::LPS28DFWData(data)
    {
    }

    LPS28DFW_2Data() {}

    static std::string header()
    {
        return "pressureTimestamp,pressure,temperatureTimestamp,temperature\n ";
    }

    void print(std::ostream& os) const
    {
        os << pressureTimestamp << "," << pressure << ","
           << temperatureTimestamp << "," << temperature << "\n";
    }
};

struct HSCMRNN015PA_1Data : Boardcore::HSCMRNN015PAData
{
    explicit HSCMRNN015PA_1Data(const Boardcore::HSCMRNN015PAData& data)
        : Boardcore::HSCMRNN015PAData(data)
    {
    }

    HSCMRNN015PA_1Data() {}

    static std::string header() { return "pressureTimestamp,pressure\n "; }

    void print(std::ostream& os) const
    {
        os << pressureTimestamp << "," << pressure << ","
           << "\n";
    }
};

struct HSCMRNN015PA_2Data : Boardcore::HSCMRNN015PAData
{
    explicit HSCMRNN015PA_2Data(const Boardcore::HSCMRNN015PAData& data)
        : Boardcore::HSCMRNN015PAData(data)
    {
    }

    HSCMRNN015PA_2Data() {}

    static std::string header() { return "pressureTimestamp,pressure\n "; }

    void print(std::ostream& os) const
    {
        os << pressureTimestamp << "," << pressure << ","
           << "\n";
    }
};

}  // namespace HILTest