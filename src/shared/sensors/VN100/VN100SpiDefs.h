/* Copyright (c) 2024 Skyward Experimental Rocketry
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

namespace Boardcore
{

namespace VN100SpiDefs
{

/**
 * @brief Internal registers definitions.
 */
enum Registers
{
    REG_MODEL_NUMBER    = 1,
    REG_QUATERNION_DATA = 9,
    REG_IMU_DATA        = 54,
};

/**
 * @brief Commands available for the sensor.
 */
enum Commands
{
    READ_REG  = 1,
    WRITE_REG = 2,
};

/**
 * @brief The expected model number to be red from the sensor.
 */
const char* const MODEL_NUMBER = "VN-100";

/**
 * @brief Size of the buffer used to retrieve the model number from the sensor.
 */
const int MODEL_NUMBER_SIZE = 24;

/**
 * @brief Size of the buffer used to retrieve imu data from the sensor.
 */
const int IMU_SAMPLE_SIZE = 44;

/**
 * @brief Size of the buffer used to retrieve quaternion data from the sensor.
 */
const int QUATERNION_SAMPLE_SIZE = 16;

}  // namespace VN100SpiDefs

}  // namespace Boardcore
