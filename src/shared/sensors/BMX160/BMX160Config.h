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

namespace Boardcore
{

/**
 * @brief BMX160 Configuration
 */
struct BMX160Config
{
    /**
     * @brief Fifo operationg mode.
     */
    enum class FifoMode
    {
        DISABLED,    ///< The fifo is completely disabled.
        HEADERLESS,  ///< Sensors MUST have the same odr.
        HEADER       ///< Sensors can have different odr.
    };

    /**
     * @brief Fifo interrupt mode.
     *
     * Uses fifo full/watermark as triggers.
     * We configure the pin as push-pull and active-low.
     */
    enum class FifoInterruptPin
    {

        DISABLED,  ///< The interrupts are completely disabled.
        PIN_INT1,  ///< Interrupts are enabled on pin 2.
        PIN_INT2,  ///< Interrupts are enabled on pin 1.
    };

    /**
     * @brief Range of the accelerometer expressed in ±g.
     */
    enum class AccelerometerRange
    {
        G_2  = 0x3,  ///< Accelaration range ±2g.
        G_4  = 0x5,  ///< Accelaration range ±4g.
        G_8  = 0x8,  ///< Accelaration range ±8g.
        G_16 = 0xC   ///< Accelaration range ±16g.
    };

    /**
     * @brief Gyroscope range expressed in °/sec.
     */
    enum class GyroscopeRange
    {
        DEG_2000 = 0x0,  ///< Gyroscope range 2000°/sec.
        DEG_1000 = 0x1,  ///< Gyroscope range 1000°/sec.
        DEG_500  = 0x2,  ///< Gyroscope range 500°/sec.
        DEG_250  = 0x3,  ///< Gyroscope range 250°/sec.
        DEG_125  = 0x4   ///< Gyroscope range 125°/sec.
    };

    /**
     * @brief Gyroscope measure unit (degrees or radiants).
     */
    enum class GyroscopeMeasureUnit
    {
        DEG,  ///< Degrees.
        RAD   ///< Radiants.
    };

    /**
     * @brief Output Data Rate expressed in Hz.
     *
     * Limits are as follows (wrong values will error the device):
     * - Accelerometer: 25/2  - 1600 (25/32 - 1600 if undersampling is enabled,
     * which is not)
     * - Gyroscope:     25    - 3200
     * - Magnetometer:  25/32 - 800
     */
    enum class OutputDataRate
    {
        HZ_0_78125 = 0x01,  ///< Output Data Rate of 25/32Hz = 0.78125.
        HZ_1_5625  = 0x02,  ///< Output Data Rate of 25/16Hz = 1.5625.
        HZ_3_125   = 0x03,  ///< Output Data Rate of 25/8Hz = 3.125.
        HZ_6_25    = 0x04,  ///< Output Data Rate of 25/4Hz = 6.25.
        HZ_12_5    = 0x05,  ///< Output Data Rate of 25/2Hz = 12.5.
        HZ_25      = 0x06,  ///< Output Data Rate of 25Hz.
        HZ_50      = 0x07,  ///< Output Data Rate of 50Hz.
        HZ_100     = 0x08,  ///< Output Data Rate of 100Hz.
        HZ_200     = 0x09,  ///< Output Data Rate of 200Hz.
        HZ_400     = 0x0A,  ///< Output Data Rate of 400Hz.
        HZ_800     = 0x0B,  ///< Output Data Rate of 800Hz.
        HZ_1600    = 0x0C,  ///< Output Data Rate of 1600Hz.
        HZ_3200    = 0x0D,  ///< Output Data Rate of 3200Hz.
    };

    /**
     * @brief Bandwidth parameter.
     *
     * For more detailed explanation, check the 2.4.1 Data Processing
     * Accelerometer and 2.4.2 Data Processing Gyroscope chapters of the BMX160
     * datasheet.
     */
    enum class BandwidthParameter
    {
        NORMAL = 0x20,  ///< Normal filter operation.
        OSR2   = 0x10,  ///< Oversampling rate of 2.
        OSR4   = 0x00,  ///< Oversampling rate of 4.
    };

    /**
     * @brief Interrupt pin mode.
     *
     * Configuration for interrupt pin behaviour.
     */
    enum class IntMode
    {
        PUSH_PULL,   //< Push-pull behaviour.
        OPEN_DRAIN,  //< Open drain behaviour.
    };

    /**
     * @brief Fifo watermark to use, in multiples of 4.
     *
     * Only values between [0-250] make sense to use.
     * A really high watermark value (the default) will disable it, falling back
     * to FIFO full.
     */
    uint8_t fifoWatermark = -1;

    /**
     * @brief Repetitions for the XY axis.
     *
     * Repetitions represent how many internal samples are averaged in order to
     * get the final outputted sample, these presets are the ones recommended
     * in the BMX160 datasheet, for more informations consult the BMM150
     * datasheet, chapter 4.2.4 Active mode.
     *
     * This are the reccomended presets:
     * - 0x01, RMS Noise (x/y/z) 1.0/1.0/1.4, Current: 0.17mA (Low power)
     * - 0x04, RMS Noise (x/y/z) 0.6/0.6/0.6, Current: 0.5mA  (Regular)
     * (Default)
     * - 0x07, RMS Noise (x/y/z) 0.5/0.5/0.5, Current: 0.8mA  (Enhanced
     * regular)
     * - 0x17, RMS Noise (x/y/z) 0.3/0.3/0.3, Current: 4.9mA  (High accuracy)
     */
    uint8_t magnetometerRepetitionsXY = 0x04;

    /**
     * @brief Repetitions for the Z axis.
     *
     * Repetitions represent how many internal samples are averaged in order to
     * get the final outputted sample, these presets are the ones reccomended
     * in the BMX160 datasheet, for more informations consult the BMM150
     * datasheet, chapter 4.2.4 Active mode.
     *
     * This are the reccomended presets:
     * - 0x02, RMS Noise (x/y/z) 1.0/1.0/1.4, Current: 0.17mA (Low power)
     * - 0x0E, RMS Noise (x/y/z) 0.6/0.6/0.6, Current: 0.5mA  (Regular)
     * (Default)
     * - 0x1A, RMS Noise (x/y/z) 0.5/0.5/0.5, Current: 0.8mA  (Enhanced
     * regular)
     * - 0x52, RMS Noise (x/y/z) 0.3/0.3/0.3, Current: 4.9mA  (High accuracy)
     */
    uint8_t magnetometerRepetitionsZ = 0x0E;

    /**
     * @brief Enable magnetometer data compensation.
     *
     * The magnetomer support compensation, but it's not documented, and the
     * current implementation is based on the source of the bosch driver.
     *
     * The implementation is slow, probably buggy, and only god knows what it
     * does, it's so bad that I added a switch to disable it, use it wisely!
     */
    bool enableCompensation = true;

    /**
     * @brief Divide the temperature sampling rate.
     *
     * This is used to limit the sampling of the temperature, use 0 to disable
     * it completely.
     *
     * Every time you call onSimpleUpdate a value is incremented and only if
     * the value is a multiple of this parameter a read is performed.
     */
    int temperatureDivider = 0;

    /**
     * @brief Should the fifo use accelerometer filtered data?
     */
    bool fifoAccelerometerFiltered = true;

    /**
     * @brief Fifo accelerometer downsampling (between 0 and 15).
     */
    uint8_t fifoAccelerationDownsampling = 0;

    /**
     * @brief Should the fifo use gyroscope filtered data?
     */
    bool fifoGyroscopeFiltered = true;

    /**
     * @brief Fifo gyroscope downsampling (between 0 and 15).
     */
    uint8_t fifoGyroscopeDownsampling = 0;

    FifoMode fifoMode              = FifoMode::DISABLED;
    FifoInterruptPin fifoInterrupt = FifoInterruptPin::DISABLED;

    OutputDataRate accelerometerDataRate      = OutputDataRate::HZ_100;
    BandwidthParameter accelerometerBandwidth = BandwidthParameter::NORMAL;
    AccelerometerRange accelerometerRange     = AccelerometerRange::G_2;

    OutputDataRate gyroscopeDataRate      = OutputDataRate::HZ_100;
    BandwidthParameter gyroscopeBandwidth = BandwidthParameter::NORMAL;
    GyroscopeRange gyroscopeRange         = GyroscopeRange::DEG_2000;
    GyroscopeMeasureUnit gyroscopeUnit    = GyroscopeMeasureUnit::RAD;

    OutputDataRate magnetometerRate = OutputDataRate::HZ_100;

    IntMode interrupt1Mode = IntMode::PUSH_PULL;
    IntMode interrupt2Mode = IntMode::PUSH_PULL;

    BMX160Config() {}
};

}  // namespace Boardcore
