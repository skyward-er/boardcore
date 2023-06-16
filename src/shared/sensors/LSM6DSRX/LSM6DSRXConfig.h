/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Fabrizio Monti
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

#include <stdint.h>

namespace Boardcore
{

struct LSM6DSRXConfig
{

    /**
     * @brief Output data rate definitions for the accelerometer.
     */
    enum class ACC_ODR : uint8_t
    {
        POWER_DOWN = 0,
        HZ_1_6     = 11,  ///< Output data rate of 1.6 Hz
        HZ_12_5    = 1,   ///< Output data rate of 12.5 Hz
        HZ_26      = 2,
        HZ_52      = 3,
        HZ_104     = 4,
        HZ_208     = 5,
        HZ_416     = 6,
        HZ_833     = 7,
        HZ_1660    = 8,
        HZ_3330    = 9,
        HZ_6660    = 10,  ///< Output data rate of 6.66 kHz
    };

    /**
     * @brief Fullscale values for the accelerometer.
     */
    enum class ACC_FULLSCALE : uint8_t
    {
        G2  = 0,
        G4  = 2,
        G8  = 3,
        G16 = 1,
    };

    /**
     * @brief Data update mode for the sensor.
     */
    enum class BDU : uint8_t
    {
        CONTINUOUS_UPDATE = 0,
        UPDATE_AFTER_READ = 1,  ///< Output registers are not updated until MSB
                                ///< and LSB have been read
    };

    /**
     * @brief Operating mode for the sensor.
     *
     * The sensor operates in 4 modes: power_down, low_power, normal,
     * high_performance. high_performance is valid for all the odr values. If
     * NORMAL mode is selected, the sensor behaviour depends on the selected
     * odr.
     */
    enum class OPERATING_MODE : uint8_t
    {
        HIGH_PERFORMANCE = 0,  ///< Valid for all odrs
        NORMAL = 1,  ///< Works in low power or normal mode depending on the odr
    };

    BDU bdu;                   ///< Data update mode.
    ACC_ODR odrAcc;            ///< Accelerometer odr.
    OPERATING_MODE opModeAcc;  ///< Operating mode for the accelerometer.
    ACC_FULLSCALE fsAcc;       ///< Fullscale selection for the accelerometer.
};

}  // namespace Boardcore