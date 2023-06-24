/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <drivers/i2c/I2C.h>
#include <sensors/Sensor.h>

#include "LPS28DFWData.h"

namespace Boardcore
{

/**
 * @brief Driver for LPS28DFW STMicroelectronics digital pressure sensor.
 */
class LPS28DFW : public Sensor<LPS28DFWData>
{
public:
    /**
     * @brief Enumeration for the full scale range to set on the sensor.
     * Available are 1260 hPa or 4060 hPa.
     */
    enum FullScaleRange : uint8_t
    {
        FS_1260,
        FS_4060
    };

    /**
     * @brief Enumeration for Output Data Rate Configuration.
     *
     * Available are One shot (only one sample calculated when signal sent), 1
     * Hz to 200 Hz. ODR configuration is valid for both pressure and
     * temperature.
     */
    enum ODR : uint8_t
    {
        ONE_SHOT = 0b0000 << 3,
        ODR_1    = 0b0001 << 3,
        ODR_4    = 0b0010 << 3,
        ODR_10   = 0b0011 << 3,
        ODR_25   = 0b0100 << 3,
        ODR_50   = 0b0101 << 3,
        ODR_75   = 0b0110 << 3,
        ODR_100  = 0b0111 << 3,
        ODR_200  = 0b1000 << 3
    };

    /**
     * @brief Oversampling average values.
     *
     * The value read from the sensor will actually be the average of multiple
     * samples. Available are from 4 to 512 averaged samples.
     *
     * @warning For an AGV value of 512, 128, 64 the maximum ODR values are
     * respectively of 25, 75 and 100 Hz. For any other AVG value all ODR
     * configurations are possible.
     */
    enum AVG : uint8_t
    {
        AVG_4   = 0b000,
        AVG_8   = 0b001,
        AVG_16  = 0b010,
        AVG_32  = 0b011,
        AVG_64  = 0b100,
        AVG_128 = 0b101,
        AVG_512 = 0b111
    };

    /**
     * @brief Struct that sums up all the settings of the sensor.
     */
    struct Config
    {
        /**
         * Last bit of the slave address; tells if the SA0 pin on the sensor is
         * connected to GND or VDD (3.3V, not 5V!).
         */
        bool sa0;
        FullScaleRange fsr;
        ODR odr = ODR::ODR_25;
        AVG avg = AVG::AVG_512;
    };

    /**
     * @brief Constructor that stores the initial settings (without applying
     * them to the sensor).
     * @param i2c I2C Peripheral.
     * @param sensorConfig Sensor configuration.
     */
    LPS28DFW(I2C& i2c, Config sensorConfig);

    /**
     * @brief Initializes the sensor with the current settings.
     * @return true if initialization succeeded, false otherwise.
     */
    bool init() override;

    /**
     * @brief The self test method returns true if we read the right whoami
     * value. We can't make a better self test due to the fact that the sensor
     * doesn't support this feature.
     * @return true if the right whoami has been read.
     */
    bool selfTest() override;

    /**
     * @brief Sets and saves the configurations passed on the parameters.
     */
    bool setConfig(const Config& config);

    /**
     * @brief Sets and saves the oversampling on the sensor.
     * @return True if setting succeeded, false otherwise.
     */
    bool setAverage(AVG avg);

    /**
     * @brief Sets and saves the output data rate.
     * @return True if setting succeeded, false otherwise.
     */
    bool setOutputDataRate(ODR odr);

    /**
     * @brief Sets and saves the full scale range.
     * @return True if setting succeeded, false otherwise.
     */
    bool setFullScaleRange(FullScaleRange fs);

private:
    LPS28DFWData sampleImpl() override;

    I2C& i2c;
    I2CDriver::I2CSlaveConfig i2cConfig;
    Config config;

    bool isInitialized = false;

    float pressureSensitivity;  ///< pressure sensitivity [LSB/Pa]

    PrintLogger logger = Logging::getLogger("lps28dfw");
};

}  // namespace Boardcore