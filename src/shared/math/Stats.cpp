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

#include "Stats.h"
#include <algorithm>
#include <cmath>
#include <limits>

using namespace std;

ostream& operator<<(ostream& os, const StatsResult& sr)
{
    os << "min=" << sr.minValue << " max=" << sr.maxValue << " mean=" << sr.mean
       << " stdev=" << sr.stdev;
    return os;
}

//
// class Stats
//

Stats::Stats()
    : minValue(numeric_limits<float>::max()),
      maxValue(numeric_limits<float>::lowest()), mean(0.f), m2(0.f), n(0)
{
}

void Stats::add(float data)
{
    if (isnan(data))
        return;

    minValue = min(minValue, data);
    maxValue = max(maxValue, data);

    // Stable algorithm for computing variance, see wikipedia
    n++;
    float delta = data - mean;
    mean += delta / n;
    m2 += delta * (data - mean);
}

void Stats::reset()
{
    minValue = numeric_limits<float>::max();
    maxValue = numeric_limits<float>::lowest();
    mean     = 0.f;
    m2       = 0.f;
    n        = 0;
}

StatsResult Stats::getStats() const
{
    switch (n)
    {
        case 0:
            return {NAN, NAN, NAN, NAN, n};
        case 1:
            return {minValue, maxValue, mean, NAN, n};
        default:
            return {minValue, maxValue, mean, sqrtf(m2 / (n - 1)), n};
    }
}

#ifdef STATS_TESTCASE

#include <fstream>
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc != 2)
        return -1;
    ifstream in(argv[1]);
    Stats stats;
    float f;
    while (in >> f)
        stats.add(f);
    cout << stats.getStats() << endl;
}

#endif  // STATS_TESTCASE
