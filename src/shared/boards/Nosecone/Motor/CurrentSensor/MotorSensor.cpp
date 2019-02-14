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

#include "MotorSensor.h"

namespace NoseconeBoard
{

MotorSensor::MotorSensor() 
{
    status.max_current_sensed = 0;
    status.min_current_sensed = 0xFFFF;
    log();

    adc.init(); 
}


void MotorSensor::run()
{
    /* Sample sensor */
    adc.updateParams();
    uint16_t adcval = adc.getValue();

    /* Update status */
    if(adcval > status.max_current_sensed)
        status.max_current_sensed = adcval;

    if(adcval < status.min_current_sensed)
        status.min_current_sensed = adcval;

    status.last_current_sensed = adcval;
    log();
}


float MotorSensor::adcToI(uint16_t adc_in)
{
    float v    = (adc_in * 3.3f) / 4096;
    float iout = v / 525;
    return (iout - 0.000030) * 10000;
}

} /* namespace NoseconeBoard */