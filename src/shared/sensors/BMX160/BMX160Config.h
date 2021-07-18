/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Davide Mor
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

/// @brief BMX160 Configuration
struct BMX160Config
{
    BMX160Config() {}

    /// @brief Fifo operating mode.
    enum class FifoMode
    {

        DISABLED,    ///< The fifo is completely disabled.
        HEADERLESS,  ///< Sensors MUST have the same odr.
        HEADER       ///< Sensors can have different odr.
    };

    /// @brief Fifo interrupt mode.
    /// Uses fifo full/watermark as triggers.
    /// We configure the pin as push-pull and active-low.
    enum class FifoInt
    {

        DISABLED,  ///< The interrupts are completely disabled.
        PIN_INT1,  ///< Interrupts are enabled on pin 2.
        PIN_INT2,  ///< Interrupts are enabled on pin 1.
    };

    /// @brief Range of the accelerometer expressed in +/- g.
    enum class AccRange
    {
        G_2  = 0x3,
        G_4  = 0x5,
        G_8  = 0x8,
        G_16 = 0xC
    };

    /// @brief Gyroscope range expressed in °/sec.
    enum class GyrRange
    {
        DEG_2000 = 0x0,
        DEG_1000 = 0x1,
        DEG_500  = 0x2,
        DEG_250  = 0x3,
        DEG_125  = 0x4
    };

    /// @brief Gyroscope measure unit (deg or rad)
    enum class GyrMeasureUnit
    {
        DEG,
        RAD
    };

    /// @brief ODR expressed in Hz.
    ///
    /// Limits are as follows (wrong values will error the device):
    /// - Accelerometer: 25/2  - 1600 (25/32 - 1600 if undersampling is enabled,
    /// which is not)
    /// - Gyroscope:     25    - 3200
    /// - Magnetometer:  25/32 - 800
    ///
    /// HZ_25_xx indicate fractional frequencies, given by: 25/xx.
    enum class Odr
    {
        HZ_25_32 = 0x01,
        HZ_25_16 = 0x02,
        HZ_25_8  = 0x03,
        HZ_25_4  = 0x04,
        HZ_25_2  = 0x05,
        HZ_25    = 0x06,
        HZ_50    = 0x07,
        HZ_100   = 0x08,
        HZ_200   = 0x09,
        HZ_400   = 0x0a,
        HZ_800   = 0x0b,
        HZ_1600  = 0x0c,
        HZ_3200  = 0x0d,
    };

    /// @brief Bandwidth parameter.
    ///
    /// For more detailed explanation, check the 2.4.1 Data Processing
    /// Accelerometer and 2.4.2 Data Processing Gyroscope chapters of the BMX160
    /// datasheet.
    enum class Bwp
    {
        NORMAL = 0x02 << 4,  ///< Normal filter operation.
        OSR2   = 0x01 << 4,  ///< Oversampling rate of 2.
        OSR4   = 0x00 << 4,  ///< Oversampling rate of 4.
    };

    /// @brief Fifo watermark to use, in multiples of 4.
    ///
    /// Only values between [0-250] make sense to use.
    /// A really high watermark value (the default) will
    /// disable it, falling back to FIFO full.
    uint8_t fifo_watermark = -1;

    /// @brief Repetitions for the XY axis
    ///
    /// Repetitions represent how many internal samples are averaged in order to
    /// get the final outputted sample, these presets are the ones reccomended
    /// in the BMX160 datasheet, for more informations consult the BMM150
    /// datasheet, chapter 4.2.4 Active mode.
    ///
    /// This are the reccomended presets:
    /// - 0x01, RMS Noise (x/y/z) 1.0/1.0/1.4, Current: 0.17mA (Low power)
    /// - 0x04, RMS Noise (x/y/z) 0.6/0.6/0.6, Current: 0.5mA  (Regular)
    /// (Default)
    /// - 0x07, RMS Noise (x/y/z) 0.5/0.5/0.5, Current: 0.8mA  (Enhanced
    /// regular)
    /// - 0x17, RMS Noise (x/y/z) 0.3/0.3/0.3, Current: 4.9mA  (High accuracy)
    uint8_t mag_repxy = 0x04;

    /// @brief Repetitions for the Z axis
    ///
    /// Repetitions represent how many internal samples are averaged in order to
    /// get the final outputted sample, these presets are the ones reccomended
    /// in the BMX160 datasheet, for more informations consult the BMM150
    /// datasheet, chapter 4.2.4 Active mode.
    ///
    /// This are the reccomended presets:
    /// - 0x02, RMS Noise (x/y/z) 1.0/1.0/1.4, Current: 0.17mA (Low power)
    /// - 0x0E, RMS Noise (x/y/z) 0.6/0.6/0.6, Current: 0.5mA  (Regular)
    /// (Default)
    /// - 0x1A, RMS Noise (x/y/z) 0.5/0.5/0.5, Current: 0.8mA  (Enhanced
    /// regular)
    /// - 0x52, RMS Noise (x/y/z) 0.3/0.3/0.3, Current: 4.9mA  (High accuracy)
    uint8_t mag_repz = 0x0E;

    /// @brief Enable magnetometer data compensation
    ///
    /// The magnetomer support compensation, but it's not documented, and the
    /// current implementation is based on the source of the bosch driver.
    ///
    /// The implementation is slow, probably buggy, and only god knows what it
    /// does, it's so bad that I added a switch to disable it, use it wisely!
    bool enable_compensation = true;

    /// @brief Divide the temperature sampling rate.
    ///
    /// This is used to limit the sampling of the temperature, use 0 to disable
    /// it completely.
    ///
    /// Every time you call onSimpleUpdate a value is incremented and only if
    /// the value is a multiple of this parameter a read is performed.
    int temp_divider = 0;

    /// @brief Should the fifo use accelerometer filtered data?
    bool fifo_acc_filtered = true;
    /// @brief Fifo accelerometer downsampling (between 0 and 15).
    uint8_t fifo_acc_downs = 0;

    /// @brief Should the fifo use gyroscope filtered data?
    bool fifo_gyr_filtered = true;
    /// @brief Fifo gyroscope downsampling (between 0 and 15).
    uint8_t fifo_gyr_downs = 0;

    FifoMode fifo_mode = FifoMode::DISABLED;
    FifoInt fifo_int   = FifoInt::DISABLED;

    Odr acc_odr        = Odr::HZ_100;
    Bwp acc_bwp        = Bwp::NORMAL;
    AccRange acc_range = AccRange::G_2;

    Odr gyr_odr             = Odr::HZ_100;
    Bwp gyr_bwp             = Bwp::NORMAL;
    GyrRange gyr_range      = GyrRange::DEG_2000;
    GyrMeasureUnit gyr_unit = GyrMeasureUnit::DEG;

    Odr mag_odr = Odr::HZ_100;
};