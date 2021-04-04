/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "BME280Data.h"
#include "drivers/spi/SPIDriver.h"
#include "sensors/Sensor.h"

class BME280 : public Sensor<BME280Data>
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

    enum FilterCoeff
    {
        FILTER_OFF      = 0x0,  ///< Filter off
        FILTER_COEFF_2  = 0x1,  ///< Filter coefficient = 2
        FILTER_COEFF_4  = 0x2,  ///< Filter coefficient = 4
        FILTER_COEFF_8  = 0x3,  ///< Filter coefficient = 8
        FILTER_COEFF_16 = 0x4   ///< Filter coefficient = 16
    };

    enum StandbyTime
    {
        STB_TIME_0_5  = 0x0,  ///< 0.5 ms
        STB_TIME_62_5 = 0x1,  ///< 62.5 ms
        STB_TIME_125  = 0x2,  ///< 125 ms
        STB_TIME_250  = 0x3,  ///< 250 ms
        STB_TIME_500  = 0x4,  ///< 500 ms
        STB_TIME_1000 = 0x5,  ///< 1000 ms
        STB_TIME_10   = 0x6,  ///< 10 ms
        STB_TIME_20   = 0x7   ///< 20 ms
    };

    union BME280Config
    {
        struct
        {
            uint8_t : 5;
            Oversampling osrs_h : 3;  ///< Oversampling of humidity
            uint8_t : 4;
            /**
             * '1' whenever a conversion is running, '0' when the result have
             * been transferred to the data registers
             */
            unsigned measuring : 1;
            uint8_t : 2;
            /**
             * '1' when the NVM data are being copied to image registers, '0'
             * when the copying is done
             */
            unsigned im_update : 1;
            Oversampling osrs_t : 3;  ///< Oversampling of temperature
            Oversampling osrs_p : 3;  ///< Oversampling of pressure
            Mode mode : 2;            ///< Device modes
            StandbyTime t_sb : 3;     ///< Inactive duration in normal mode
            FilterCoeff filter : 3;   ///< Time constant of the IIR filter
            uint8_t : 1;
            unsigned spi3w_en : 1;  ///< Enables 3-wire SPI interface
        } bits;

        struct
        {
            uint8_t ctrl_hum;   ///< Humidity options
            uint8_t status;     ///< Device status
            uint8_t ctrl_meas;  ///< Pressure and temperature options
            uint8_t config;     ///< Rate, filter and interface options
        } bytes;

        uint8_t bytes_array[4];
    };

    union BME280RawData
    {
        struct
        {
            uint32_t pressure : 24;
            uint32_t temperature : 24;
            uint16_t humidity;
        } bits;

        struct
        {
            uint8_t press_msb;
            uint8_t press_lsb;
            uint8_t press_xlsb;
            uint8_t temp_msb;
            uint8_t temp_lsb;
            uint8_t temp_xlsb;
            uint8_t hum_msb;
            uint8_t hum_lsb;
        } bytes;
    };

    struct BME280Comp
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
        uint8_t dig_H1;
        int16_t dig_H2;
        int8_t dig_H3;
        int16_t dig_H4 : 12;
        int16_t dig_H5 : 12;
        int8_t dig_H6;
    };

    enum BME280Registers : uint8_t
    {
        REG_CALIB_0 = 0x88,
        // Calibration register 1-25

        REG_ID    = 0xD0,
        REG_RESET = 0xE0,

        REG_CALIB_26 = 0xE1,
        // Calibration register 27-41

        REG_CTRL_HUM  = 0xF2,
        REG_STATUS    = 0xF3,
        REG_CTRL_MEAS = 0xF4,
        REG_CONFIG    = 0xF5,

        REG_PRESS_MSB  = 0xF7,
        REG_PRESS_LSB  = 0xF8,
        REG_PRESS_XLSB = 0xF9,
        REG_TEMP_MSB   = 0xFA,
        REG_TEMP_LSB   = 0xFB,
        REG_TEMP_XLSB  = 0xFC,
        REG_HUM_MSB    = 0xFD,
        REG_HUM_LSB    = 0xFE,
    };

    static constexpr uint8_t WHO_AM_I_REG = 0xD0;
    static constexpr uint8_t WHO_AM_I_VAL = 0x60;

    BME280(SPISlave spiSlave_);

    /**
     * @brief Initialize the device with the specified configuration
     */
    bool init() override;

    /**
     * @brief Reads the WHO AM I register
     *
     * @return True if everything ok
     */
    bool selfTest() override;

private:
    BME280Data sampleImpl() override;

    void setConfiguration();

    void loadCompensationParameters();

    // Compensation algorithm rev.1.1 from Bosh datasheet

    int32_t getFineTemperature(int32_t adc_T);

    int32_t compensateTemperature(int32_t t_fine);

    uint32_t compensatePressure(int32_t adc_P);

    uint32_t compensateHumidity(int32_t adc_H);

    /**
     * @brief Check the WHO AM I code from the device.
     *
     * @return true if the device is recognized
     */
    bool checkWhoAmI();

    const SPISlave spiSlave;
    BME280Config config;
    BME280Comp compParams;
    int32_t t_fine;  // Used in compensation algorithm

    bool initialized;  // Whether the sensor has been initialized
};
