/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>

#include "BMP280Data.h"

namespace Boardcore
{

class BMP280 : public Sensor<BMP280Data>
{
public:
    enum Oversampling
    {
        SKIPPED         = 0x0,  ///< Skipped (output set to 0x8000)
        OVERSAMPLING_1  = 0x1,  ///< Oversampling x1
        OVERSAMPLING_2  = 0x2,  ///< Oversampling x2
        OVERSAMPLING_4  = 0x3,  ///< Oversampling x4
        OVERSAMPLING_8  = 0x4,  ///< Oversampling x8
        OVERSAMPLING_16 = 0x5,  ///< Oversampling x16
    };

    enum Mode
    {
        SLEEP_MODE  = 0x0,  ///< Sleep mode
        FORCED_MODE = 0x1,  ///< Forced mode
        NORMAL_MODE = 0x3   ///< Normal mode
    };

    enum StandbyTime
    {
        STB_TIME_0_5  = 0x0,  ///< 0.5 ms
        STB_TIME_62_5 = 0x1,  ///< 62.5 ms
        STB_TIME_125  = 0x2,  ///< 125 ms
        STB_TIME_250  = 0x3,  ///< 250 ms
        STB_TIME_500  = 0x4,  ///< 500 ms
        STB_TIME_1000 = 0x5,  ///< 1000 ms
        STB_TIME_20   = 0x6,  ///< 20 ms
        STB_TIME_40   = 0x7   ///< 40 ms
    };

    enum FilterCoeff
    {
        FILTER_OFF      = 0x0,  ///< Filter off
        FILTER_COEFF_2  = 0x1,  ///< Filter coefficient = 2
        FILTER_COEFF_4  = 0x2,  ///< Filter coefficient = 4
        FILTER_COEFF_8  = 0x3,  ///< Filter coefficient = 8
        FILTER_COEFF_16 = 0x4   ///< Filter coefficient = 16
    };

    union BMP280Config
    {
        struct __attribute__((packed)) BMP280ConfigBits
        {
            // status
            /**
             * '1' when the NVM data are being copied to image registers, '0'
             * when the copying is done
             */
            uint8_t imUpdate : 1;
            uint8_t : 2;
            /**
             * '1' whenever a conversion is running, '0' when the result have
             * been transferred to the data registers
             */
            uint8_t measuring : 1;
            uint8_t : 4;

            Mode mode : 2;  ///< Device modes
            Oversampling
                oversamplingPressure : 3;  ///< Oversampling of pressure
            Oversampling
                oversamplingTemperature : 3;  ///< Oversampling of temperature

            // config
            uint8_t spi3wEn : 1;  ///< Enables 3-wire SPI interface
            uint8_t : 1;
            FilterCoeff filter : 3;       ///< Time constant of the IIR filter
            StandbyTime standbyTime : 3;  ///< Inactive duration in normal mode
        } bits;

        struct
        {
            uint8_t status;                      ///< Device status
            uint8_t ctrlPressureAndTemperature;  ///< Pressure and temperature
                                                 ///< options
            uint8_t config;  ///< Rate, filter and interface options
        } bytes;

        uint8_t bytesArray[3];
    };

    union BMP280Comp
    {
        struct __attribute__((packed))
        {
            uint16_t dig_T1;
            int16_t dig_T2;
            int16_t dig_T3;
            uint16_t dig_P1;
            int16_t dig_P2;
            int16_t dig_P3;
            int16_t dig_P4;
            int16_t dig_P5;
            int16_t dig_P6;
            int16_t dig_P7;
            int16_t dig_P8;
            int16_t dig_P9;
        } bits;

        uint8_t bytesArray[24];
    };

    static constexpr uint8_t REG_ID_VAL = 0x58;  ///< Who am I value

    ///< Default register values
    static const BMP280Config BMP280_DEFAULT_CONFIG;

    ///< Datasheet values for indoor navigation
    static const BMP280Config BMP280_CONFIG_ALL_ENABLED;

    ///< Temperature enabled in forced mode
    static const BMP280Config BMP280_CONFIG_TEMP_SINGLE;

    explicit BMP280(SPISlave spiSlave,
                    BMP280Config config = BMP280_CONFIG_ALL_ENABLED);

    /**
     * @brief Initialize the device with the specified configuration
     */
    bool init() override;

    /**
     * @brief Sets the sensor mode
     *
     * Values:
     * - SLEEP_MODE: No measurements are performed
     * - FORCED_MODE: A single measurement is performed when this function
     * writes the configuration, after the measurement the sensor return to
     * sleep mode
     * - NORMAL_MODE: Automated cycling between measurements, standby time can
     * be set using setStandbyTime()
     */
    void setSensorMode(Mode mode);

    /**
     * @brief Sets the oversampling for pressure readings, use SKIPPED to
     * disable pressure sampling
     */
    void setPressureOversampling(Oversampling oversampling);

    /**
     * @brief Sets the oversampling for temperature readings, use SKIPPED to
     * disable temperature sampling
     */
    void setTemperatureOversampling(Oversampling oversampling);

    /**
     * @brief Sets the coefficient for the IIR filter (applied to temperature
     * and pressure)
     */
    void setFilterCoeff(FilterCoeff filterCoeff);

    /**
     * @brief Sets the standby time between readings in normal mode
     */
    void setStandbyTime(StandbyTime standbyTime);

    /**
     * @brief Reads only the pressure, does not set the configuration
     */
    PressureData readPressure();

    /**
     * @brief Reads only the temperature, does not set the configuration
     */
    TemperatureData readTemperature();

    /**
     * @brief Maximum measurement time formula from datasheet page 51
     *
     * @return Time in milliseconds
     */
    static unsigned int calculateMaxMeasurementTime(BMP280Config config);

    unsigned int getMaxMeasurementTime();

    /**
     * @brief Reads the WHO AM I register
     *
     * @return True if everything ok
     */
    bool selfTest() override;

private:
    BMP280Data sampleImpl() override;

    void reset();

    void setConfiguration();

    void setConfiguration(BMP280Config config);

    BMP280Config readConfiguration();

    void loadCompensationParameters();

    // Compensation algorithm rev.1.1 from Bosh datasheet

    int32_t computeFineTemperature(int32_t adcTemperature);

    int32_t compensateTemperature(int32_t fineTemperature);

    uint32_t compensatePressure(int32_t adcPressure);
    /**
     * @brief Check the WHO AM I code from the device.
     *
     * @return true if the device is recognized
     */
    bool checkWhoAmI();

    enum Registers : uint8_t
    {
        REG_CALIB_0 = 0x08,
        // Calibration register 1-25

        REG_ID    = 0x50,
        REG_RESET = 0x60,

        REG_STATUS    = 0x73,
        REG_CTRL_MEAS = 0x74,
        REG_CONFIG    = 0x75,

        REG_PRESS_MSB  = 0x77,
        REG_PRESS_LSB  = 0x78,
        REG_PRESS_XLSB = 0x79,
        REG_TEMP_MSB   = 0x7A,
        REG_TEMP_LSB   = 0x7B,
        REG_TEMP_XLSB  = 0x7C
    };

    const SPISlave spiSlave;
    BMP280Config config;
    BMP280Comp compParams;
    int32_t fineTemperature;  // Used in compensation algorithm

    PrintLogger logger = Logging::getLogger("bmp280");
};

}  // namespace Boardcore
