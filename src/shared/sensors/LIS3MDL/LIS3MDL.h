/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Riccardo Musso
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
#include <drivers/timer/TimestampTimer.h>
#include <sensors/Sensor.h>

#include "LIS3MDLData.h"
#include "miosix.h"

namespace Boardcore
{

/**
 * Driver for LIS3MDL, a three-axis magnetic sensor.
 */
class LIS3MDL : public Sensor<LIS3MDLData>
{
public:
    /**
     * Constants for Output Data Rate configuration.
     *
     * Note: constants values already include
     * axis operative mode selection and FAST_ODR
     * options, so they are ready to be put inside
     * CTRL_REG1 along with TEMP_EN and ST
     */
    enum ODR : uint8_t
    {
        ODR_0_625_HZ = 0x00,  //!< 0.625 Hz
        ODR_1_25_HZ  = 0x04,  //!<  1.25 Hz
        ODR_2_5_HZ   = 0x08,  //!<   2.5 Hz
        ODR_5_HZ     = 0x0c,  //!<     5 Hz
        ODR_10_HZ    = 0x10,  //!<    10 Hz
        ODR_20_HZ    = 0x14,  //!<    20 Hz
        ODR_40_HZ    = 0x18,  //!<    40 Hz
        ODR_80_HZ    = 0x1c,  //!<    80 Hz

        ODR_155_HZ  = 0x62,  //!<    155 Hz
        ODR_300_HZ  = 0x42,  //!<    300 Hz
        ODR_560_HZ  = 0x22,  //!<    560 Hz
        ODR_1000_HZ = 0x02,  //!<   1000 Hz

        /**
         * Constant used by the driver: this bit
         * is 1 for ODR > 80 Hz, set to 0 otherwise.
         */
        FAST_ODR_BIT = 0x02,
    };

    enum FullScale : uint8_t
    {
        FS_4_GAUSS  = 0x00,  //!<  +/- 4  gauss
        FS_8_GAUSS  = 0x20,  //!<  +/- 8  gauss
        FS_12_GAUSS = 0x40,  //!<  +/- 12 gauss
        FS_16_GAUSS = 0x60,  //!<  +/- 16 gauss
    };

    /**
     * @brief Operative mode constants
     *
     * Operative mode (that is power consumpion) options.
     * Higher power implies better performance.
     */
    enum OperativeMode : uint8_t
    {
        OM_LOW_POWER        = 0x0,
        OM_MEDIUM_POWER     = 0x1,
        OM_HIGH_POWER       = 0x2,
        OM_ULTRA_HIGH_POWER = 0x3,
    };

    /**
     * @brief Sensor configuration
     *
     * This struct contains all the settings the user
     * is able to modify with the help of the driver.
     * They are applied in the constructor of LIS3MDL class
     * and on each call of LIS3MDL::applyConfig()
     */
    struct Config
    {
        /**
         * @brief Constructor
         * Creates an instance with the default settings.
         */
        Config() {}

        /**
         * Full scale setting
         * @see LIS3MDL::FullScale
         */
        FullScale scale = FS_8_GAUSS;

        /**
         * @brief Data rate configuration
         *
         * Default: 40 Hz
         *
         * Important: if ODR is set more than 80 Hz,
         * operative mode of x and y axis will be set
         * accordingly.
         *
         * @see LIS3MDL::ODR
         */
        ODR odr = ODR_40_HZ;

        /**
         * Operative mode for x and y axis.
         * Note: if ODR is greater than 80 Hz,
         * this setting will be ignored and actual
         * operative mode will be set depending of
         * the chosen frequency.
         * Default: ultra high performance
         *
         * @see LIS3MDL::OperativeMode
         */
        OperativeMode xyMode = OM_ULTRA_HIGH_POWER;

        /**
         * Operative mode for z axis.
         * Default: ultra high performance
         *
         * @see LIS3MDL::OM
         */
        OperativeMode zMode = OM_ULTRA_HIGH_POWER;

        /**
         * Enables temperature sensor.
         * Default: true
         */
        bool enableTemperature = true;

        /**
         * @brief Sets the value of tempDivider.
         *
         * With the given value you can instruct the driver to update
         * the temperature according to a different rate.
         * The temperature will be updated only once in `tempDivider` calls
         * to sampleImpl(), so for example:
         * 2 -> updated half the times,
         * 1 -> updated every time.
         * By default the divider is set to 1
         *
         */
        unsigned temperatureDivider = 1;

        /**
         * @brief Enables interrupts
         *
         * Whether are interrupts enabled respectively
         * on the x, y and z axis. If it is set to true
         * on at least one axis, the interrupts will be
         * generated otherwise, they will be completely
         * disabled by the driver.
         *
         * Default: disabled on all axis.
         */
        bool enableInterrupt[3] = {false, false, false};

        /**
         * Absolute value of the threshold that triggers the interrupt
         * (expressed in gauss). Default: 0
         */
        float threshold = 0;

        /**
         * @brief BDU setting
         *
         * If set to true, the sensor won't update the data
         * until it is read, if false the sensor data will be
         * continously updated. Default: false
         */
        bool doBlockDataUpdate = false;
    };

    /**
     * Constructs the default config for SPI Bus.
     *
     * @returns the default SPIBusConfig
     */
    static SPIBusConfig getDefaultSPIConfig()
    {
        SPIBusConfig config{};
        config.clockDivider = SPI::ClockDivider::DIV_32;
        return config;
    }

    /**
     * @brief The constructor.
     *
     * Takes all relevant SPI info and the settings
     * for the sensor. If no SPIBusConfig is given, it will
     * be used the default configuration.
     *
     * @param bus The SPI interface
     * @param pin The CS pin
     * @param spiConfig SPI configuration, optional
     * @param config Driver configuration, optional
     *
     * @see LIS3MDL::Config
     */
    LIS3MDL(SPIBusInterface& bus, miosix::GpioPin pin,
            SPIBusConfig spiConfig = getDefaultSPIConfig(), Config config = {})
        : mSlave{bus, pin, spiConfig}, mConfig(config), currDiv(0),
          isInitialized(false)
    {
    }

    bool init() override
    {
        if (isInitialized)
        {
            LOG_ERR(logger, "Attempted to initialized sensor twice but failed");
            last_error = ALREADY_INIT;
            return false;
        }

        {
            SPITransaction spi(mSlave);
            uint8_t res = spi.readRegister(WHO_AM_I);

            if (res != WHO_AM_I_VALUE)
            {
                LOG_ERR(logger,
                        "WHO_AM_I value differs from expectation: read 0x{02x} "
                        "but expected 0x{02x}",
                        res, WHO_AM_I_VALUE);
                last_error = INVALID_WHOAMI;
                return false;
            }
        }

        isInitialized = true;
        return applyConfig(mConfig);
    }

    /**
     * @brief Executes self test
     *
     * The init() method must have been called before.
     *
     * @returns false if the sensor failed the self test, true otherwise.
     */
    bool selfTest() override
    {
        if (!isInitialized)
        {
            LOG_ERR(logger, "Invoked selfTest() but sensor was unitialized");
            last_error = NOT_INIT;
            return false;
        }

        /*
         * NUM_SAMPLES: number of samples used
         * to take the average value before tests.
         * NUM_TESTS: number of actual tests.
         * SLEEP_TIME: millis between samples/tests
         */
        constexpr int NUM_SAMPLES = 5, NUM_TESTS = 5, SLEEP_TIME = 50;

        /*
         * Absolute value of extra tolerance
         */
        constexpr float t = 0.1f;

        /*
         * Range which delta must be between, one for
         * axis and expressed as {min, max}. The unit is gauss
         */
        constexpr float r[3][2] = {{1.f, 3.f}, {1.f, 3.f}, {0.1f, 1.f}};

        float avg_x = 0.f, avg_y = 0.f, avg_z = 0.f;

        {
            SPITransaction spi(mSlave);
            spi.writeRegister(CTRL_REG2, FS_12_GAUSS);
        }
        updateUnit(FS_12_GAUSS);

        for (int i = 0; i < NUM_SAMPLES; ++i)
        {
            miosix::Thread::sleep(SLEEP_TIME);

            LIS3MDLData lastData = sampleImpl();
            avg_x += lastData.mag_x;
            avg_y += lastData.mag_y;
            avg_z += lastData.mag_z;
        }

        avg_x /= NUM_SAMPLES;
        avg_y /= NUM_SAMPLES;
        avg_z /= NUM_SAMPLES;

        /*
         * Setting up the sensor settings for
         * proper usage of the self test mode.
         */
        {
            SPITransaction spi(mSlave);

            spi.writeRegister(CTRL_REG1, ODR_20_HZ | ENABLE_SELF_TEST |
                                             (OM_ULTRA_HIGH_POWER << 4));
            spi.writeRegister(CTRL_REG2, FS_12_GAUSS);
            spi.writeRegister(CTRL_REG4, OM_ULTRA_HIGH_POWER << 2);
        }

        /*
         * Deltas: absolute difference between
         * the values measured before and during
         * selftest
         */
        float d[3];

        for (int i = 0; i < NUM_TESTS; ++i)
        {
            miosix::Thread::sleep(SLEEP_TIME);

            LIS3MDLData lastData = sampleImpl();
            d[0]                 = std::abs(lastData.mag_x - avg_x);
            d[1]                 = std::abs(lastData.mag_y - avg_y);
            d[2]                 = std::abs(lastData.mag_z - avg_z);

            bool passed = true;
            for (int j = 0; j < 3; ++j)
            {
                if (d[j] < (r[j][0] - t) || d[j] > (r[j][1] + t))
                    passed = false;
            }

            if (!passed)
            {
                // reset configuration, then return
                applyConfig(mConfig);

                last_error = SELF_TEST_FAIL;
                return false;
            }
        }

        return applyConfig(mConfig);
    }

    /**
     * @brief Overwrites the sensor settings.
     *
     * Writes a certain config to the sensor
     * registers. This method is automatically
     * called in LIS3MDL::init() using as parameter
     * the configuration given in the constructor.
     *
     * This method checks if the values were actually
     * written in the sensor's registers and returns false
     * if at least one of them was not as expected.
     *
     * @param config The configuration to be applied
     * @returns true if the configuration was applied successfully,
     * false otherwise.
     */
    bool applyConfig(Config config)
    {

        SPITransaction spi(mSlave);
        uint8_t reg = 0, err = 0;

        mConfig = config;
        currDiv = 0;

        /*  -- CTRL_REG1 --  */
        if (config.enableTemperature)
        {
            reg = ENABLE_TEMPERATURE;
        }
        reg |= config.odr;

        // odr <= 80Hz
        if (!(config.odr & FAST_ODR_BIT))
            reg |= config.xyMode << 4;
        spi.writeRegister(CTRL_REG1, reg);
        err |= spi.readRegister(CTRL_REG1) != reg;

        /* -- CTRL_REG2 -- */
        reg = config.scale;
        spi.writeRegister(CTRL_REG2, reg);
        err |= spi.readRegister(CTRL_REG2) != reg;

        /* -- CTRL_REG3 -- */
        reg = CONTINOUS_CONVERSION;
        spi.writeRegister(CTRL_REG3, reg);
        err |= spi.readRegister(CTRL_REG3) != reg;

        /* -- CTRL_REG4 -- */
        reg = config.zMode << 2;
        spi.writeRegister(CTRL_REG4, reg);
        err |= spi.readRegister(CTRL_REG4) != reg;

        /* -- CTRL_REG5 -- */
        if (config.doBlockDataUpdate)
        {
            reg = ENABLE_BDU;
        }
        else
        {
            reg = 0;
        }

        spi.writeRegister(CTRL_REG5, reg);
        err |= spi.readRegister(CTRL_REG5) != reg;

        /* -- INT_CFG -- */
        if (config.enableInterrupt[0])
        {
            reg = ENABLE_INT_X;
        }
        else
        {
            reg = 0;
        }
        if (config.enableInterrupt[1])
        {
            reg |= ENABLE_INT_Y;
        }
        if (config.enableInterrupt[2])
        {
            reg |= ENABLE_INT_Z;
        }

        // the interrupt of at least one axis is enabled
        if (reg)
        {
            reg |= ENABLE_INT_PIN;
        }

        reg |= 0x08;
        spi.writeRegister(INT_CFG, reg);
        err |= spi.readRegister(INT_CFG) != reg;

        /** INT_THS */
        uint16_t val =
            static_cast<uint16_t>(std::abs(config.threshold / mUnit));
        reg = static_cast<uint8_t>(0xff & val);
        spi.writeRegister(INT_THS_L, reg);
        err |= spi.readRegister(INT_THS_L) != reg;

        reg = static_cast<uint8_t>(val >> 8);
        reg &=
            0x7f;  // remove MSB (according to the datasheet, it must be zero)
        spi.writeRegister(INT_THS_H, reg);
        err |= spi.readRegister(INT_THS_H) != reg;

        /* Set mUnit according to scale */
        updateUnit(config.scale);

        if (err)
        {
            LOG_ERR(logger, "Spi error");
            last_error = BUS_FAULT;
            return false;
        }

        return true;
    }

private:
    /**
     * @brief Reads data from the sensor
     *
     * The init method must have been called before.
     * Output data can be fetched with the methods
     * compassDataPtr() and tempDataPtr inherited respectevely
     * from CompassSensor and TemperatureSensor classes.
     * Important: the temperature will be taken only once in a while
     * according to the value of `temperatureDivider`
     *
     * @returns false if the sensor was unitialized, true otherwise.
     */
    LIS3MDLData sampleImpl() override
    {
        if (!isInitialized)
        {
            LOG_ERR(logger,
                    "Invoked sampleImpl() but sensor was "
                    "unitialized");
            last_error = NOT_INIT;
            return last_sample;
        }

        SPITransaction spi(mSlave);

        if (!spi.readRegister(STATUS_REG))
        {
            last_error = NO_NEW_DATA;
            return last_sample;
        }

        // Reset any error
        last_error = SensorErrors::NO_ERRORS;

        int16_t val;
        LIS3MDLData newData{};

        if (mConfig.enableTemperature)
        {
            if (currDiv == 0)
            {
                val = spi.readRegister(TEMP_OUT_L);
                val |= spi.readRegister(TEMP_OUT_H) << 8;

                newData.temp_timestamp = TimestampTimer::getTimestamp();
                newData.temp = static_cast<float>(val) / LSB_PER_CELSIUS +
                               REFERENCE_TEMPERATURE;
            }
            else
            {
                // Keep old value
                newData.temp = last_sample.temp;
            }

            currDiv = (currDiv + 1) % mConfig.temperatureDivider;
        }

        newData.mag_timestamp = TimestampTimer::getTimestamp();

        val = spi.readRegister(OUT_X_L);
        val |= spi.readRegister(OUT_X_H) << 8;
        newData.mag_x = mUnit * val;

        val = spi.readRegister(OUT_Y_L);
        val |= spi.readRegister(OUT_Y_H) << 8;
        newData.mag_y = mUnit * val;

        val = spi.readRegister(OUT_Z_L);
        val |= spi.readRegister(OUT_Z_H) << 8;
        newData.mag_z = mUnit * val;

        return newData;
    }

    SPISlave mSlave;
    Config mConfig;

    unsigned currDiv;
    bool isInitialized;
    float mUnit = 0;

    void updateUnit(FullScale fs)
    {
        switch (fs)
        {
            case FS_4_GAUSS:
                mUnit = 1.f / LSB_PER_GAUSS_FS_4;
                break;

            case FS_8_GAUSS:
                mUnit = 1.f / LSB_PER_GAUSS_FS_8;
                break;

            case FS_12_GAUSS:
                mUnit = 1.f / LSB_PER_GAUSS_FS_12;
                break;

            case FS_16_GAUSS:
                mUnit = 1.f / LSB_PER_GAUSS_FS_16;
                break;
        }
    };

    /**
     * List of addresses of sensor registers
     */
    enum Reg : uint8_t
    {
        WHO_AM_I = 0x0f,

        CTRL_REG1 = 0x20,
        CTRL_REG2 = 0x21,
        CTRL_REG3 = 0x22,
        CTRL_REG4 = 0x23,
        CTRL_REG5 = 0x24,

        STATUS_REG = 0x27,
        OUT_X_L    = 0x28,
        OUT_X_H    = 0x29,
        OUT_Y_L    = 0x2a,
        OUT_Y_H    = 0x2b,
        OUT_Z_L    = 0x2c,
        OUT_Z_H    = 0x2d,

        TEMP_OUT_L = 0x2e,
        TEMP_OUT_H = 0x2f,

        INT_CFG   = 0x30,
        INT_THS_L = 0x32,
        INT_THS_H = 0x33,
    };

    /**
     * Misc. constants used by the driver. They are not
     * particularly useful to the user.
     */
    enum Constants : unsigned
    {
        WHO_AM_I_VALUE = 0x3d,

        CONTINOUS_CONVERSION  = 0x0,
        REFERENCE_TEMPERATURE = 25,
        LSB_PER_CELSIUS       = 8,

        LSB_PER_GAUSS_FS_4  = 6842,
        LSB_PER_GAUSS_FS_8  = 3421,
        LSB_PER_GAUSS_FS_12 = 2281,
        LSB_PER_GAUSS_FS_16 = 1711,

        ENABLE_TEMPERATURE = 0x80,
        ENABLE_SELF_TEST   = 0x01,
        ENABLE_BDU         = 0x40,

        ENABLE_INT_PIN = 0x01,
        ENABLE_INT_X   = 0x80,
        ENABLE_INT_Y   = 0x40,
        ENABLE_INT_Z   = 0x20,
    };

    PrintLogger logger = Logging::getLogger("lis3mdl");
};

}  // namespace Boardcore
