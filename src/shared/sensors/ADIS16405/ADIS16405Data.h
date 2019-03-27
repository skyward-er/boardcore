/* 
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Nuno Barcellos
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

#ifndef ADIS16405DATA_H
#define ADIS16405DATA_H

#include <cstdint>

/*
 * Burst data collection. This establishes right datatype for 
 * the registers because trying to work with 12 or 14 bit twos
 * complement that doesn't sign extend to 16 bits is unpleasant.
 */
struct ADIS16405Data
{
    uint16_t supply_out;  //  Power supply measurement
    int16_t xgyro_out;    //  X-axis gyroscope output
    int16_t ygyro_out;    //  Y-axis gyroscope output
    int16_t zgyro_out;    //  Z-axis gyroscope output
    int16_t xaccl_out;    //  X-axis accelerometer output
    int16_t yaccl_out;    //  Y-axis accelerometer output
    int16_t zaccl_out;    //  Z-axis accelerometer output
    int16_t xmagn_out;    //  X-axis magnetometer measurement
    int16_t ymagn_out;    //  Y-axis magnetometer measurement
    int16_t zmagn_out;    //  Z-axis magnetometer measurement
    int16_t temp_out;     //  Temperature output
    uint16_t aux_adc;     //  Auxiliary ADC measurement
};

#endif