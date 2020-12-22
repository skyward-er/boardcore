/**
 * Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <math/Vec3.h>
#include <stdio.h>

/// @brief Output from the BMX160 magnetometer.
struct BMX160Mag
{
    Vec3 data;           ///< Data expressed in μT.
    uint32_t timestamp;  ///< Timestamp of the data in microseconds.

    /// @brief Debug function for printing the data.
    void print()
    {
        TRACE("Mag [%.4f s]:\t%.2f\t%.2f\t%.2f\n", timestamp / 1000000.0f,
              data.getX(), data.getY(), data.getZ());
    }
};

/// @brief Output from the BMX160 accelerometer.
struct BMX160Acc
{
    Vec3 data;           ///< Data expressed in g.
    uint32_t timestamp;  ///< Timestamp of the data in microseconds.

    /// @brief Debug function for printing the data.
    void print()
    {
        TRACE("Acc [%.4f s]:\t%.2f\t%.2f\t%.2f\n", timestamp / 1000000.0f,
              data.getX(), data.getY(), data.getZ());
    }
};

/// @brief Output from the BMX160 Gyroscope.
struct BMX160Gyr
{
    Vec3 data;           ///< Data expressed in °/s.
    uint32_t timestamp;  ///< Timestamp of the data in microseconds.

    /// @brief Debug function for printing the data.
    void print()
    {
        TRACE("Gyr [%.4f s]:\t%.2f\t%.2f\t%.2f\n", timestamp / 1000000.0f,
              data.getX(), data.getY(), data.getZ());
    }
};

/// @brief Class representing a BMX160 Data fifo
template <typename T, int N>
class BMX160Fifo
{
public:
    friend class BMX160;

    BMX160Fifo() : len(0), idx(0) {}

    /// Pop a single sample from the fifo
    T pop()
    {
#ifdef DEBUG
        assert(idx < N && "FIFO buffer underflow");
#endif
        return data[idx++];
    }

    /// Get the remaining count of samples in the fifo
    int count() { return len; }

private:
    /// Push a single sample in the fifo (should not be called outside driver)
    void push(T sample)
    {
#ifdef DEBUG
        assert(len < N && "FIFO buffer overflow");
#endif
        data[len++] = sample;
    }

    /// Clear every sample in the fifo (should not be called outside driver)
    void clear() { len = idx = 0; }

    T data[N];
    int len;
    int idx;
};