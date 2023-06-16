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
 * @brief Driver for LPS28DFW STMicroelectronics digital pressure sensor working
 * in I2C.
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
     * @brief Enumeration for the output data rate to set on the sensor.
     * Available are One shot (only one sample calculated when signal sent), 1
     * Hz to 200 Hz.
     */
    enum ODR : uint8_t
    {
        ONE_SHOT = (0b0000 << 3),
        ODR_1    = (0b0001 << 3),
        ODR_4    = (0b0010 << 3),
        ODR_10   = (0b0011 << 3),
        ODR_25   = (0b0100 << 3),
        ODR_50   = (0b0101 << 3),
        ODR_75   = (0b0110 << 3),
        ODR_100  = (0b0111 << 3),
        ODR_200  = (0b1000 << 3)
    };

    /**
     * @brief Enumeration for the oversampling to set on the sensor. The value
     * read from the sensor will actually be the average of multiple samples.
     * Available are from 4 to 512 averaged samples.
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
     * @brief Working mode of the sensor. Available ones are one shot mode
     * (samples acquired by the sensors only upon an explicit request) and
     * continuous mode (samples calculated continuously).
     */
    enum Mode
    {
        ONE_SHOT_MODE,   // BYPASS, ODR = ONE_SHOT
        CONTINUOUS_MODE  // BYPASS, ODR = odr
    };

    /**
     * @brief Struct that sums up all the settings of the sensor.
     */
    typedef struct
    {
        bool sa0;  ///< Last bit of the slave address; tells if the SA0 pin on
                   ///< the sensor is connected to GND or VDD (3.3V, not 5V!).
        FullScaleRange fsr;  ///< Full scale range
        AVG avg;             ///< Average avg samples
        Mode mode;           ///< Mode of operation
        ODR odr;             ///< Output data rate
        bool drdy;           ///< Enable Interrupt for Data Ready
    } SensorConfig;

    /**
     * @brief Constructor that stores the initial settings (without applying
     * them to the sensor).
     * @param i2c I2C Peripheral that will be used to communicate with the
     * sensor.
     * @param sensorConfig Configuration of the sensor with ODR, AVG, FSR,
     * interrupt enabled and fifo size.
     */
    LPS28DFW(I2C& i2c, SensorConfig sensorConfig);

    /**
     * @brief Initializes the sensor with the current settings.
     */
    bool init() override;

    /**
     * @brief Sets and saves the configurations passed on the parameters.
     */
    bool setConfig(const SensorConfig& config);

    /**
     * @brief The self test method returns true if we read the right whoami
     * value. We can't make a better self test due to the fact that the sensor
     * doesn't support this feature.
     * @return true if the right whoami has been written.
     */
    bool selfTest() override;

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

    /**
     * @brief Sets and saves the full scale range.
     * @return True if setting succeeded, false otherwise.
     */
    bool setDRDYInterrupt(bool drdy);

private:
    LPS28DFWData sampleImpl() override;

    /**
     * @brief Converting the bytes read from the sensor to the pressure in Pa.
     */
    float convertPressure(uint8_t pressXL, uint8_t pressL, uint8_t pressH);

    /**
     * @brief Converting the bytes read from the sensor to the temperature in
     * °C.
     */
    float convertTemperature(uint8_t tempL, uint8_t tempH);

    I2CDriver::I2CSlaveConfig i2cConfig;  ///< I2C address and speed mode
    I2C& i2c;                             ///< I2C bus on which the sensor lays
    SensorConfig sensorConfig;            ///< Sensor settings
    bool isInitialized =
        false;  ///< Flag to tell if the sensor has been initialized
    uint16_t pressureSensitivity;  ///< pressure sensitivity [LSB/Pa]
    PrintLogger logger = Logging::getLogger("lps28dfw");
};

}  // namespace Boardcore