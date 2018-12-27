/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
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

#include <miosix.h>

#include "ActiveObject.h"
#include "drivers/adc/ADC.h"

#include "CurrentStatus.h"

#include <boards/Nosecone/LogProxy/LogProxy.h>

namespace NoseconeBoard
{

// typedef miosix::Gpio<GPIOF_BASE, 8> sensor_l;
typedef miosix::Gpio<GPIOF_BASE, 6> hbridge_current_sensor;

using ADC_t = SensorADC<1, 5, hbridge_current_sensor>;

static const int SAMPLE_FREQ   = 10;
static const int SAMPLE_PERIOD = 1000 / SAMPLE_FREQ;

static ADC_t adc(SAMPLE_FREQ);

/**
 * Simple driver for reading the current value in the H-Bridge.
 */
class MotorSensor : public ActiveObject
{

public:
    /**
     * @brief Init ADC to read the current value.
     */
    MotorSensor();

    /**
     * @brief Helper function to convert raw adc value, for debug pourposes.
     */
    float adcToI(uint16_t adc_in);

protected:
    /**
     * @brief Sensor sampling function, inherited from ActiveObject. 
     */
    void run() override;

private:
    CurrentStatus status;

    /**
     * @brief Save the status.
     */
    inline void log()
    {
        Singleton<LoggerProxy>::getInstance()->log(status);
    }
};

} /* namespace NoseconeBoard */