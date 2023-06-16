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

namespace LSM6DSRXDefs
{

/// @brief Fifo max size expressed as number of samples.
const uint16_t FIFO_SIZE = 200;

/// @brief Sensor who_am_i register value.
const uint8_t WHO_AM_I_VALUE = 0x6B;

/// @brief Lower bound value for accelerometer self test.
const float ACC_SELF_TEST_MIN = 40.0;
/// @brief Upper bound value for accelerometer self test.
const float ACC_SELF_TEST_MAX = 1700.0;
/// @brief Lower bound value for gyroscope self test. Valid for self test
/// fullscale = 2000dps.
const float GYR_SELF_TEST_MIN = 1500.0;
/// @brief Upper bound value for gyroscope self test.  Valid for self test
/// fullscale = 2000dps.
const float GYR_SELF_TEST_MAX = 7000.0;

/**
 * @brief Internal registers definitions.
 */
enum Registers
{
    REG_WHO_AM_I = 0x0F,  ///< who_am_i register

    REG_CTRL1_XL = 0x10,  ///< accelerometer control register
    REG_CTRL2_G  = 0x11,  ///< gyroscope control register
    REG_CTRL3_C  = 0x12,  ///< set bdu
    REG_CTRL6_C  = 0x15,  ///< enable/disable high performance mode for the
                          ///< accelerometer
    REG_CTRL7_G = 0x16,   ///< enable/disable high performance mode for the
                          ///< gyroscope

    REG_FIFO_CTRL1 = 0x07,
    REG_FIFO_CTRL2 = 0x08,

    REG_FIFO_CTRL3 = 0x09,  ///< fifo control register 3 (select batch data
                            ///< rate for gyro and acc)
    REG_FIFO_CTRL4 = 0x0A,  ///< fifo control register 4 (select fifo mode,
                            ///< batch data rate for temperature sensor and the
                            ///< decimation factor for timestamp batching)

    REG_FIFO_STATUS1 =
        0x3A,  ///< Gives number of unread sensor data stored in FIFO.
    REG_FIFO_STATUS2 = 0x3B,  ///< Gives number of unread sensor data and
                              ///< the current status (watermark, overrun,
                              ///< full, BDR counter) of the FIFO.

    REG_STATUS   = 0x1E,  ///< data ready register.
    REG_CTRL4_C  = 0x13,
    REG_CTRL5_C  = 0x14,
    REG_CTRL8_XL = 0x17,
    REG_CTRL9_XL = 0x18,
    REG_CTRL10_C = 0x19,

    REG_FIFO_DATA_OUT_TAG = 0x78,
    REG_FIFO_DATA_OUT_X_L = 0x79,
    REG_FIFO_DATA_OUT_X_H = 0x7A,
    REG_FIFO_DATA_OUT_Y_L = 0x7B,
    REG_FIFO_DATA_OUT_Y_H = 0x7C,
    REG_FIFO_DATA_OUT_Z_L = 0x7D,
    REG_FIFO_DATA_OUT_Z_H = 0x7E,

    REG_OUTX_L_A =
        0x28,  ///< Low bits output register for the accelerometer (x axis)
    REG_OUTX_H_A =
        0x29,  ///< High bits output register for the accelerometer (x axis)
    REG_OUTY_L_A =
        0x2A,  ///< Low bits output register for the accelerometer (y axis)
    REG_OUTY_H_A =
        0x2B,  ///< High bits output register for the accelerometer (y axis)
    REG_OUTZ_L_A =
        0x2C,  ///< Low bits output register for the accelerometer (z axis)
    REG_OUTZ_H_A =
        0x2D,  ///< High bits output register for the accelerometer (z axis)

    REG_OUTX_L_G =
        0x22,  ///< Low bits output register for the gyroscope (x axis)
    REG_OUTX_H_G =
        0x23,  ///< High bits output register for the gyroscope (x axis)
    REG_OUTY_L_G =
        0x24,  ///< Low bits output register for the gyroscope (y axis)
    REG_OUTY_H_G =
        0x25,  ///< High bits output register for the gyroscope (y axis)
    REG_OUTZ_L_G =
        0x26,  ///< Low bits output register for the gyroscope (z axis)
    REG_OUTZ_H_G =
        0x27,  ///< High bits output register for the gyroscope (z axis)

    REG_TIMESTAMP0 = 0x40,
    REG_TIMESTAMP1 = 0x41,
    REG_TIMESTAMP2 = 0x42,
    REG_TIMESTAMP3 = 0x43,

    REG_INTERNAL_FREQ_FINE = 0x63,

    REG_INT1_CTRL = 0x0D,  ///< set interrupts on INT1 pin.
    REG_INT2_CTRL = 0x0E,  ///< set interrupts on INT2 pin.
};

}  // namespace LSM6DSRXDefs

}  // namespace Boardcore
