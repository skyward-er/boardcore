/* Computes statistics (min, max, mean and variance) on a dataset
 *
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Author: Federico Terraneo
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

#include <ostream>

/**
 * Statisitics computed by the Stats class
 */
struct StatsResult
{
    float minValue;  ///< Min value found so far
    float maxValue;  ///< Max value found so far
    float mean;      ///< Mean of datased
    float stdev;     ///< Standard deviation of datset
    float nSamples   ///< Number of samples
};

/**
 * Allows printing StatsResult to an ostream
 */
std::ostream& operator<<(std::ostream& os, const StatsResult& sr);

/**
 * Computes on-line statisitics of a dataset. This class should theoretically
 * work with datasets of up to 2^32-1 elements
 */
class Stats
{
public:
    /**
     * Constructor
     */
    Stats();

    /**
     * Add an element
     */
    void add(float data);

    /**
     * Reset all the stats
     */
    void reset();
    
    /**
     * Return statistics of the elements added so far
     */
    StatsResult getStats() const;

private:
    float minValue, maxValue, mean, m2;
    unsigned int n;
};
