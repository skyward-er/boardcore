/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Conterio
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <sensors/Sensor.h>

/**
 * @brief Class used to simulate a differential pressure sensor in software.
 *
 * It takes the pressure measures of two sensors (one for the total pressure
 * and one for the static one) and returns the difference between the two
 * pressure values.
 */
template <typename FirstPressureData, typename SecondPressureData>
class SoftwareDifferentialPressureSensor : public Sensor<PressureData>
{
    static_assert(
        checkIfProduces<Sensor<FirstPressureData>, PressureData>::value,
        "First template argument must be a sensor that produces pressure "
        "data.");
    static_assert(
        checkIfProduces<Sensor<SecondPressureData>, PressureData>::value,
        "Second template argument must be a sensor that produces pressure "
        "data.");

public:
    SoftwareDifferentialPressureSensor(
        Sensor<FirstPressureData>* first_pressure_sensor,
        Sensor<SecondPressureData>* second_pressure_sensor)
        : first_pressure_sensor(first_pressure_sensor),
          second_pressure_sensor(second_pressure_sensor)
    {
    }

    bool init() override { return true; }

    bool selfTest() override { return true; };

    /**
     * @return the difference between the two pressure values
     */
    PressureData sampleImpl() override
    {
        float p1 = first_pressure_sensor->getLastSample().press;
        float p2 = second_pressure_sensor->getLastSample().press;

        return PressureData{miosix::getTick(), p1 - p2};
    }

private:
    Sensor<FirstPressureData>* first_pressure_sensor;
    Sensor<SecondPressureData>* second_pressure_sensor;
};