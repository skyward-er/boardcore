/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Conterio
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
#include <math.h>
#include <sensors/Sensor.h>

#include "LIS3DSHData.h"

namespace Boardcore
{

/**
 * Driver for stm32f407vg discovery on-board 3-axis
 * accelerometer + temperature sensor.
 *
 * The sensor is connected to SPI1 using the
 * following GPIOs: PA5 : clock
 *                  PA6 : miso
 *                  PA7 : mosi
 *                  PE3 : chip select
 */
class LIS3DSH : public Sensor<LIS3DSHData>
{
public:
    /**
     *  @brief Constructor.
     *
     *  @param bus          the spi bus.
     *  @param chipSelect  the chipSelect for the sensor.
     *  @param odr         output data rate for the accelerometer.
     *                      Default value is 100 Hz.
     *  @param bdu         BlockDataUpdate value, continuous or non-continuous
     *                      update mode. Default value is to update after data
     *                      has been read (BDU=1).
     *  @param fullScale  full scale range (from +/-2g up to +/-16g).
     *                      Default value is +/-2g.
     */
    LIS3DSH(SPIBusInterface& bus, miosix::GpioPin chipSelect,
            uint8_t odr       = OutputDataRate::ODR_100_HZ,
            uint8_t bdu       = BlockDataUpdate::UPDATE_AFTER_READ_MODE,
            uint8_t fullScale = FullScale::FULL_SCALE_2G)
        : spiSlave(bus, chipSelect), odr(odr), bdu(bdu), fullScale(fullScale)
    {
        spiSlave.config.clockDivider =
            SPI::ClockDivider::DIV_64;  // used to set the spi baud rate
                                        // (maximum is 10 Mhz)
    }

    /**
     *  @brief Constructor.
     *
     *  @param bus          the spi bus.
     *  @param chipSelect  the chipSelect for the sensor.
     *  @param config       the spi bus configurations.
     *  @param odr         output data rate for the accelerometer.
     *                      Default value is 100 Hz.
     *  @param bdu         BlockDataUpdate value, continuous or non-continuous
     *                      update mode. Default value is to update after data
     *                      has been read (BDU=1).
     *  @param fullScale  full scale range (from +/-2g up to +/-16g).
     *                      Default value is +/-2g.
     */
    LIS3DSH(SPIBusInterface& bus, miosix::GpioPin chipSelect,
            SPIBusConfig config, uint8_t odr = OutputDataRate::ODR_100_HZ,
            uint8_t bdu       = BlockDataUpdate::UPDATE_AFTER_READ_MODE,
            uint8_t fullScale = FullScale::FULL_SCALE_2G)
        : spiSlave(bus, chipSelect, config), odr(odr), bdu(bdu),
          fullScale(fullScale)
    {
    }

    /**
     * @brief Initialize the sensor.
     *
     * @return boolean value indicating whether the operation succeded or not
     */
    bool init() override
    {
        // check if the sensor is already initialized
        if (initialized)
        {
            LOG_WARN(logger, "Already initialized");
            lastError = SensorErrors::ALREADY_INIT;
            return false;
        }

        // check if the sensor is working properly
        if (!checkWhoAmI())
        {                  // whoami default value
            return false;  // sensor correctly initialized
        }

        SPITransaction spi(spiSlave);

        // set the full scale value in CTRL_REG5
        uint8_t ctrlReg5Value = (fullScale << 3);
        spi.writeRegister(CTRL_REG5, ctrlReg5Value);

        // select the correct sensitivity
        // for the specified full scale range
        sensitivity = selectSensitivity();

        // set the output data rate and the BDU in CTRL_REG4
        // the three least significant bits are enable bits for X, Y and Z axis
        uint8_t ctrlReg4Value =
            (odr << 4) | (bdu << 3) | (7 << 0);  // 7 = 111 -> enable the 3 axis
        spi.writeRegister(CTRL_REG4, ctrlReg4Value);

        initialized = true;

        return true;
    }

    /**
     * @brief Check if the sensor is working.
     *
     * @return boolean indicating whether the sensor is correctly working or not
     */
    bool selfTest() override
    {
        // check if the sensor is initialized
        if (!initialized)
        {
            LOG_WARN(logger,
                     "Unable to perform selftest, sensor not initialized");
            lastError = SensorErrors::NOT_INIT;
            return false;
        }

        const uint8_t numSamples = 5;  // number of samples to be used
        // vectors for storing samples, both
        // in self-test and no-self-test modes
        float X_ST[numSamples]    = {0};
        float Y_ST[numSamples]    = {0};
        float Z_ST[numSamples]    = {0};
        float X_NO_ST[numSamples] = {0};
        float Y_NO_ST[numSamples] = {0};
        float Z_NO_ST[numSamples] = {0};
        // vectors containing avg values for each axis
        float AVG_ST[3]    = {0};  // one element per axis
        float AVG_NO_ST[3] = {0};  // one element per axis

        // set output data rate to 50 hz
        uint8_t ctrlReg4Value = (OutputDataRate::ODR_100_HZ << 4) |
                                (BlockDataUpdate::UPDATE_AFTER_READ_MODE << 3) |
                                (7 << 0);

        {
            SPITransaction spi(spiSlave);
            spi.writeRegister(CTRL_REG4, ctrlReg4Value);
        }

        // set full scale to default value +/-2g
        // enable the self-test mode with positive sign
        uint8_t ctrlReg5Value = (FullScale::FULL_SCALE_2G << 3) | (1 << 1);

        {
            SPITransaction spi(spiSlave);
            spi.writeRegister(CTRL_REG5, ctrlReg5Value);
        }

        // read samples in self-test positive sign mode
        for (uint8_t i = 0; i < numSamples; i++)
        {
            AccelerometerData accelData = readAccelData();
            X_ST[i]                     = accelData.accelerationX;
            Y_ST[i]                     = accelData.accelerationY;
            Z_ST[i]                     = accelData.accelerationZ;
            miosix::Thread::sleep(10);
        }
        // reset the self-test bits
        ctrlReg5Value &= ~(3 << 1);
        // normal mode with full scale range +/-2g
        ctrlReg5Value |= (FULL_SCALE_2G << 3);

        {
            SPITransaction spi(spiSlave);
            spi.writeRegister(CTRL_REG5, ctrlReg5Value);
        }

        // read samples in normal mode
        for (uint8_t i = 0; i < numSamples; i++)
        {
            AccelerometerData accelData = readAccelData();
            X_NO_ST[i]                  = accelData.accelerationX;
            Y_NO_ST[i]                  = accelData.accelerationY;
            Z_NO_ST[i]                  = accelData.accelerationZ;
            miosix::Thread::sleep(10);
        }
        // compute averages vectors:
        // they contain one element for each axis
        // (position 0 for x, 1 for y and 2 for z)
        // AVG_ST    : for self-test samples
        // AVG_NO_ST : for normal mode samples
        for (uint8_t i = 0; i < numSamples; i++)
        {
            AVG_ST[0] += X_ST[i];
            AVG_ST[1] += Y_ST[i];
            AVG_ST[2] += Z_ST[i];
            AVG_NO_ST[0] += X_NO_ST[i];
            AVG_NO_ST[1] += Y_NO_ST[i];
            AVG_NO_ST[2] += Z_NO_ST[i];
        }
        for (uint8_t i = 0; i < 3; i++)
        {
            AVG_ST[i] /= numSamples;
            AVG_NO_ST[i] /= numSamples;
        }

        // Reset registers values with the ones
        // specified in the constructor:
        // set the output data rate value in CTRL_REG4
        ctrlReg4Value = (odr << 4) | (bdu << 3) | (7 << 0);

        {
            SPITransaction spi(spiSlave);
            spi.writeRegister(CTRL_REG4, ctrlReg4Value);
        }
        // set the full scale value in CTRL_REG5
        ctrlReg5Value = (fullScale << 3);  // normal mode

        {
            SPITransaction spi(spiSlave);
            spi.writeRegister(CTRL_REG5, ctrlReg5Value);
        }

        float delta[3] = {0};
        for (uint8_t i = 0; i < 3; i++)
        {
            delta[i] = fabs(AVG_NO_ST[i] - AVG_ST[i]);
        }

        LOG_INFO(logger,
                 "Selftest: delta[x] = {}, delta[y] = {}, delta[z] = {}",
                 delta[0], delta[1], delta[2]);

        // check that the averages differences
        // do not exceed maximum tolerance
        if ((delta[0] >
             SELF_TEST_DIFF_X_Y + SELF_TEST_DIFF_X_Y * SELF_TEST_TOLERANCE) ||
            (delta[1] >
             SELF_TEST_DIFF_X_Y + SELF_TEST_DIFF_X_Y * SELF_TEST_TOLERANCE) ||
            (delta[2] >
             SELF_TEST_DIFF_Z + SELF_TEST_DIFF_Z * SELF_TEST_TOLERANCE))
        {
            LOG_ERR(logger, "Selftest failed");
            lastError = SensorErrors::SELF_TEST_FAIL;
            return false;
        }

        return true;
    }

    /**
     * @brief Output data rate allowed values (4 bits).
     */
    enum OutputDataRate
    {
        ODR_POWER_DOWN = 0,  // 0000
        ODR_3_125_HZ   = 1,  // 0001, 3.125 Hz
        ODR_6_25_HZ    = 2,  // 0010, 6.25  Hz
        ODR_12_5_HZ    = 3,  // 0011, 12.5  Hz
        ODR_25_HZ      = 4,  // 0100
        ODR_50_HZ      = 5,  // 0101
        ODR_100_HZ     = 6,  // 0110, default value
        ODR_400_HZ     = 7,  // 0111
        ODR_800_HZ     = 8,  // 1000
        ODR_1600_HZ    = 9   // 1001
    };

    /**
     * @brief Full scale range allowed values (3 bits).
     */
    enum FullScale
    {
        FULL_SCALE_2G  = 0,  // 000, +/- 2g
        FULL_SCALE_4G  = 1,  // 001, +/- 4g
        FULL_SCALE_6G  = 2,  // 010, +/- 6g
        FULL_SCALE_8G  = 3,  // 011, +/- 8g
        FULL_SCALE_16G = 4,  // 100  +/- 16g
    };

    /**
     * @brief Block data update allowed modes (1 bit).
     */
    enum BlockDataUpdate
    {
        CONTINUOUS_UPDATE_MODE = 0,  // continuous update of accelerometer data
        UPDATE_AFTER_READ_MODE =
            1  // values updated only when MSB and LSB are read (recommended)
    };

private:
    /**
     * @brief Read new data from the accelerometer.
     *
     * Accelerations are returned in g.
     *
     * @return boolean value indicating whether the operation succeeded or not
     */
    LIS3DSHData sampleImpl() override
    {
        // check if the sensor is initialized
        if (!initialized)
        {
            LOG_WARN(logger, "Unable to sample, sensor not initialized");
            lastError = SensorErrors::NOT_INIT;
            return lastSample;
        }

        AccelerometerData accelData = readAccelData();
        TemperatureData tempData    = readTemperature();

        if (lastError != SensorErrors::NO_ERRORS)
        {
            return lastSample;
        }
        else
        {
            return LIS3DSHData(accelData, tempData);
        }
    }

    /**
     * @brief Read accelerometer data.
     *
     * @return the read accelerometer sample
     */
    AccelerometerData readAccelData()
    {
        AccelerometerData accelData;

        SPITransaction spi(spiSlave);

        // read the sensor's status register
        uint8_t status = spi.readRegister(STATUS);

        if (status & 0x08)
        {  // bit 3 of status set to 1 (new data available)
            if (status & 0x80)
            {  // bit 7 of status set to 1 (some data overwritten)

                accelData.accelerationTimestamp =
                    TimestampTimer::getTimestamp();

                // read acceleration on X
                int8_t accel_L = spi.readRegister(OUT_X_L);
                int8_t accel_H = spi.readRegister(OUT_X_H);
                accelData.accelerationX =
                    static_cast<float>(combine(accel_H, accel_L)) * sensitivity;

                // read acceleration on Y
                accel_L = spi.readRegister(OUT_Y_L);
                accel_H = spi.readRegister(OUT_Y_H);
                accelData.accelerationY =
                    static_cast<float>(combine(accel_H, accel_L)) * sensitivity;

                // read acceleration on Z
                accel_L = spi.readRegister(OUT_Z_L);
                accel_H = spi.readRegister(OUT_Z_H);
                accelData.accelerationZ =
                    static_cast<float>(combine(accel_H, accel_L)) * sensitivity;

                lastError = SensorErrors::NO_ERRORS;
            }
        }
        else
        {
            lastError = SensorErrors::NO_NEW_DATA;
        }

        return accelData;
    }

    /**
     * @brief Read temperature data.
     *
     * @return the read temperature sample
     */
    TemperatureData readTemperature()
    {
        SPITransaction spi(spiSlave);

        // the temperature is given as a 8-bits integer (in 2-complement)
        int8_t t = spi.readRegister(OUT_T);

        return TemperatureData{
            TimestampTimer::getTimestamp(),
            t + TEMPERATURE_REF};  // add the 'zero' of the temperature sensor
    }

    /**
     * @brief Check that the WHO_AM_I register
     *        contains the correct value.
     *
     * @return boolean value indicating whether the value read
     *          from the WHO_AM_I register is correct or not
     */
    bool checkWhoAmI()
    {
        SPITransaction spi(spiSlave);

        // check the WHO_AM_I_REG register
        uint8_t whoAmIValue = spi.readRegister(WHO_AM_I_REG);
        if (whoAmIValue == WHO_AM_I_DEFAULT_VALUE)
        {
            LOG_DEBUG(logger, "Correct WHO_AM_I value");
            return true;
        }
        else
        {
            LOG_ERR(logger, "Wrong WHO_AM_I value, got {} instead of {}",
                    whoAmIValue, WHO_AM_I_DEFAULT_VALUE);
            lastError = SensorErrors::INVALID_WHOAMI;
        }

        return false;
    }

    /**
     * @brief Combine low and high bits in a single number.
     *
     * @param msb   the most significatn bits
     * @param lsb   the least significant bits
     * @return MSB and LSB combined in one value
     */
    int16_t combine(uint8_t msb, uint8_t lsb) { return (msb << 8) | lsb; }

    /**
     * @brief Given the requested full scale range, select the correct
     * sensitivity value.
     *
     * @return the sensitivity value corresponding to the requested full scale
     * range
     */
    float selectSensitivity()
    {
        float s;
        switch (fullScale)
        {
            case FULL_SCALE_2G:
                s = sensitivityValues[FullScale::FULL_SCALE_2G];
                break;
            case FULL_SCALE_4G:
                s = sensitivityValues[FullScale::FULL_SCALE_4G];
                break;
            case FULL_SCALE_6G:
                s = sensitivityValues[FullScale::FULL_SCALE_6G];
                break;
            case FULL_SCALE_8G:
                s = sensitivityValues[FullScale::FULL_SCALE_8G];
                break;
            case FULL_SCALE_16G:
                s = sensitivityValues[FullScale::FULL_SCALE_16G];
                break;
            default:
                LOG_ERR(logger, "Invalid full scale range given, using +/-2g");
                this->fullScale = FullScale::FULL_SCALE_2G;
                s               = sensitivityValues[FullScale::FULL_SCALE_2G];
                break;
        }
        return s;
    }

    /**
     * @brief Registers' addresses definition.
     */
    enum REG
    {

        // whoami register
        WHO_AM_I_REG = 0x0F,

        // control registers for the accelerometer
        CTRL_REG4 =
            0x20,  // control register to set accelerometer's ODR and BDU
        CTRL_REG1 = 0x21,  // state Machine 1 interrupt configuration register
        CTRL_REG2 = 0x22,  // state Machine 2 interrupt configuration register
        CTRL_REG3 = 0x23,
        CTRL_REG5 =
            0x24,  // control register to set the accelerometer full scale
                   // range, anti-aliansing filter and self-test enable
        CTRL_REG6 = 0x25,

        // status register
        STATUS = 0x27,

        // accelerometer output registers
        // for x, y and z axis
        // (low and high bits in separate registers)
        OUT_X_L = 0x28,
        OUT_X_H = 0x29,
        OUT_Y_L = 0x2A,
        OUT_Y_H = 0x2B,
        OUT_Z_L = 0x2C,
        OUT_Z_H = 0x2D,

        // temperature output register
        OUT_T = 0x0C,
    };

    SPISlave spiSlave;

    bool initialized = false;  // whether the sensor has been initialized or not

    uint8_t odr;        // output data rate, default 100 Hz
    uint8_t bdu;        // continuous or block after update
    uint8_t fullScale;  // full scale value, default +/- 2g

    /**
     * @brief Sensitivity values corresponding to full scale range allowed
     * values.
     */
    const float sensitivityValues[5] = {0.06, 0.12, 0.18, 0.24, 0.73};

    float sensitivity =
        sensitivityValues[FullScale::FULL_SCALE_2G];  // default sensitivity
                                                      // value

    const uint8_t WHO_AM_I_DEFAULT_VALUE = 63;  // 00111111

    const float TEMPERATURE_REF = 25.0f;  // temperature sensor 'zero'/reference
                                          // value (value 0x00 from the sensor
                                          // corresponds to 25 degrees celsius)

    const float SELF_TEST_DIFF_X_Y  = 140.0f;  // 140 mg
    const float SELF_TEST_DIFF_Z    = 590.0f;  // 590 mg
    const float SELF_TEST_TOLERANCE = 0.3f;

    PrintLogger logger = Logging::getLogger("lis3dsh");
};

}  // namespace Boardcore
