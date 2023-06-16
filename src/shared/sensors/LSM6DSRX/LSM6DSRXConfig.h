/* Copyright (c) 2023 Skyward Experimental Rocketry
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
     *
     * If OPERATING_MODE is set to NORMAL, the power mode depends on the choosen
     * odr.
     */
    enum class ACC_ODR : uint8_t
    {
        POWER_DOWN = 0,   ///< Disabled
        HZ_1_6     = 11,  ///< ODR of 1.6 Hz - AVAILABLE ONLY IN LOW POWER MODE
        HZ_12_5    = 1,   ///< ODR of 12.5 Hz - low power
        HZ_26      = 2,   ///< ODR of 26 Hz - low power
        HZ_52      = 3,   ///< ODR of 52 Hz - low power
        HZ_104     = 4,   ///< ODR of 104 Hz - normal
        HZ_208     = 5,   ///< ODR of 208 Hz - normal
        HZ_416     = 6,   ///< ODR of 416 Hz - high performance
        HZ_833     = 7,   ///< ODR of 833 Hz - high performance
        HZ_1660    = 8,   ///< ODR of 1.66 kHz - high performance
        HZ_3330    = 9,   ///< ODR of 3.33 kHz - high performance
        HZ_6660    = 10,  ///< ODR of 6.66 kHz - high performance
    };

    /**
     * @brief Output data rate definitions for the gyroscope.
     *
     * If OPERATING_MODE is set to NORMAL, the power mode depends on the choosen
     * odr.
     */
    enum class GYR_ODR : uint8_t
    {
        POWER_DOWN = 0,   ///< Disabled
        HZ_12_5    = 1,   ///< ODR of 12.5 Hz - low power
        HZ_26      = 2,   ///< ODR of 26 Hz - low power
        HZ_52      = 3,   ///< ODR of 52 Hz - low power
        HZ_104     = 4,   ///< ODR of 104 Hz - normal
        HZ_208     = 5,   ///< ODR of 208 Hz - normal
        HZ_416     = 6,   ///< ODR of 416 Hz - high performance
        HZ_833     = 7,   ///< ODR of 833 Hz - high performance
        HZ_1660    = 8,   ///< ODR of 1.66 kHz - high performance
        HZ_3330    = 9,   ///< ODR of 3.33 kHz - high performance
        HZ_6660    = 10,  ///< ODR of 6.66 kHz - high performance
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
     * @brief Fullscale values for the gyroscope.
     */
    enum class GYR_FULLSCALE : uint8_t
    {
        DPS_125  = 0b0010,
        DPS_250  = 0,
        DPS_500  = 0b0100,
        DPS_1000 = 0b1000,
        DPS_2000 = 0b1100,
        DPS_4000 = 0b0001,
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

    /**
     * @brief Fifo operating mode.
     */
    enum class FIFO_MODE : uint8_t
    {
        BYPASS     = 0,  ///< Fifo disabled.
        FIFO       = 1,  ///< Stops collecting data when FIFO is full.
        CONTINUOUS = 6,  ///< If the FIFO is full, the new sample overwrites the
                         ///< older one.
    };

    /**
     * @brief Selects decimation for timestamp batching in FIFO. Write rate will
     * be the maximum rate between accelerometer and gyro BDR divided by
     * decimation decoder.
     */
    enum class FIFO_TIMESTAMP_DECIMATION : uint8_t
    {
        DEC_1 = 1,  ///< max(BDR_XL[Hz],BDR_GY[Hz]) [Hz]

        // Those options below are disabled, because without DEC_1 the driver
        // would not work properly (multiple samples would share the same
        // timestamp or have none)

        // DISABLED = 0,  ///< Timestamp not batched in FIFO.
        // DEC_8    = 2,  ///< max(BDR_XL[Hz],BDR_GY[Hz]) / 8 [Hz]
        // DEC_32   = 3,  ///< max(BDR_XL[Hz],BDR_GY[Hz]) / 32 [Hz]
    };

    /**
     * @brief Selects batch data rate in FIFO for temperature data.
     */
    enum class FIFO_TEMPERATURE_BDR : uint8_t
    {
        DISABLED = 0,  ///< Temperature not batched in FIFO.

        // commented so that temperature doesn't get sampled in fifo.
        // HZ_1_6   = 1,  ///< 1.6 Hz
        // HZ_12_5  = 2,  ///< 12.5 Hz
        // HZ_52    = 3,  ///< 52 Hz
    };

    /**
     * @brief Selectable interrupts. You can combine multiple interrupts with
     * bitwise or.
     */
    enum class INTERRUPT : uint8_t
    {
        NOTHING        = 0,   ///< no interrupt selected
        ACC_DRDY       = 1,   ///< Accelerometer data ready.
        GYR_DRDY       = 2,   ///< Gyroscope data ready.
        FIFO_THRESHOLD = 8,   ///< FIFO threshold interrupt.
        FIFO_OVERRUN   = 16,  ///< FIFO overrun interrupt.
        FIFO_FULL      = 32,  ///< FIFO full interrupt.
    };

    BDU bdu;  ///< Data update mode.

    // accelerometer
    ACC_ODR odrAcc;            ///< Accelerometer odr.
    OPERATING_MODE opModeAcc;  ///< Operating mode for the accelerometer.
    ACC_FULLSCALE fsAcc;       ///< Fullscale selection for the accelerometer.

    // gyroscope
    GYR_ODR odrGyr;            ///< Accelerometer odr.
    OPERATING_MODE opModeGyr;  ///< Operating mode for the accelerometer.
    GYR_FULLSCALE fsGyr;       ///< Fullscale selection for the accelerometer.

    // fifo
    FIFO_MODE fifoMode;  ///< Fifo operating mode.
    FIFO_TIMESTAMP_DECIMATION
    fifoTimestampDecimation;  ///< decimation for timestamp batching in
                              ///< FIFO.
    FIFO_TEMPERATURE_BDR
    fifoTemperatureBdr;  ///< batch data rate for temperature data in FIFO.

    // interrupt
    INTERRUPT int1InterruptSelection;  ///< interrupts selected on INT1 pin.
    INTERRUPT int2InterruptSelection;  ///< interrupts selected on INT2 pin.

    /**
     * @brief Fifo watermark level (expressed as a 9 bits number). Warning: this
     * number represents the number of samples extracted from the sensor's fifo,
     * not the actual number of samples that will be present inside the driver's
     * fifo `lastFifo`.
     *
     * The sensor's fifo max size is 511 samples, and those samples can be
     * accelerometer data, gyroscope data or timestamps.
     */
    uint16_t fifoWatermark;
};

}  // namespace Boardcore
