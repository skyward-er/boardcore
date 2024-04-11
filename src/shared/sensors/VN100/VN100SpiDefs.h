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
enum Registers : uint8_t
{
    REG_MODEL_NUMBER  = 1,     ///< WhoAmI register
    REG_QUAT_IMU_DATA = 15,    ///< Quaternion, accelerometer, gyroscope and
                               ///< magnetometer data register
    REG_SYNC            = 32,  ///< Used to set data ready interrupt
    REG_TEMP_PRESS_DATA = 54,  ///< Temperature and pressure data register
};

/**
 * @brief Commands available for the sensor.
 */
enum Commands : uint8_t
{
    READ_REG  = 1,
    WRITE_REG = 2,
};

/**
 * @brief Error codes of the sensor.
 */
enum class VNErrors : uint8_t
{
    NO_ERROR               = 0,
    HARD_FAULT             = 1,
    SERIAL_BUFFER_OVERFLOW = 2,
    INVALID_CHECKSUM       = 3,
    INVALID_COMMAND        = 4,
    NOT_ENOUGH_PARAMETERS  = 5,
    TOO_MANY_PARAMETERS    = 6,
    INVALID_PARAMETER      = 7,
    INVALID_REGISTER       = 8,
    UNAUTHORIZED_ACCESS    = 9,
    WATCHDOG_RESET         = 10,
    OUTPUT_BUFFER_OVERFLOW = 11,
    INSUFFICIENT_BAUDRATE  = 12,
    ERROR_BUFFER_OVERFLOW  = 255,
};

/**
 * @brief Data format of the synchronization control register, used for read
 * and write operations.
 */
struct __attribute__((packed)) SynchronizationData
{
    uint8_t syncInMode;         ///< Behaviour of the syncIn event
    uint8_t syncInEdge;         ///< Trigger syncIn on rising or falling edge
    uint16_t syncInSkipFactor;  ///< How many times trigger edges defined by
                                ///< SyncInEdge should occur prior to triggering
                                ///< a SyncIn event
    const uint32_t RESERVED = 0;  ///< Reserved, do not use
    uint8_t syncOutMode;          ///< Behavior of the SyncOut event
    uint8_t syncOutPolarity;      ///< The polarity of the output pulse on the
                                  ///< SyncOut pin (positive or negative)
    uint16_t syncOutSkipFactor;   ///< how many times the sync out event should
                                  ///< be skipped before actually triggering the
                                  ///< SyncOut pin
    uint32_t
        syncOutPulseWidth;  ///< Controls the desired width of the SyncOut pulse
    const uint32_t RESERVED2 = 0;  ///< Reserved, do not use
};

/**
 * @brief The expected model number to be red from the sensor.
 */
const char* const MODEL_NUMBER = "VN-100";

/**
 * @brief Size of the buffer used to retrieve the model number from the sensor.
 * It corresponds to the size of the register, see the datasheet for details.
 */
const int MODEL_NUMBER_SIZE = 24;

/**
 * @brief Size of the buffer used to retrieve data from the sensor.
 * It corresponds to the size of the register, see the datasheet for details.
 */
const int SAMPLE_SIZE = 52;

/**
 * @brief Size of the buffer used to retrieve temperature and pressure data
 * from the sensor. It corresponds to the size of the register, see the
 * datasheet for details.
 */
const int TEMP_PRESS_SIZE = 44;

/**
 * @brief Width of the SyncOut pulse in nanoseconds. Now is set to 1
 * millisecond.
 */
const uint32_t SYNC_OUT_PULSE_WIDTH = 1000000;

}  // namespace VN100SpiDefs

}  // namespace Boardcore
