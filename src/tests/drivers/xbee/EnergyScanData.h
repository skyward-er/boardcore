/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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

#include <array>
#include <string>
#include <ostream>

using std::array;
using std::string;
using std::to_string;

struct EnergyScanData
{
    long long timestamp;
    int channel_data[30];

    EnergyScanData() = default;

    EnergyScanData(long long ts, array<int, 30> scan)
    {
        for(int i = 0; i < 30; i++)
        {
            channel_data[i] = scan[i];
        }
    }

    static string header()
    {
        string out = "timestamp";
        for(int i = 0; i < 30; i++)
        {
            out += ",channel_" + to_string(i);
        }
        return out + "\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp;

        for(int i = 0; i < 30; i++)
        {
            os << "," << channel_data[i];
        }

        os << "\n";
    }
};