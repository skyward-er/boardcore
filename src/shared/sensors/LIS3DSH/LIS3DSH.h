/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Conterio
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

#pragma once

#include <math.h>

#include "drivers/spi/SPIDriver.h"
#include "sensors/Sensor.h"

struct LIS3DSHData : public TimestampData,
                     public AccelerometerData,
                     public TemperatureData
{
    LIS3DSHData()
        : TimestampData{miosix::getTick()}, AccelerometerData{0.0, 0.0, 0.0},
          TemperatureData{0.0}
    {
    }

    LIS3DSHData(float x, float y, float z, float temp)
        : TimestampData{miosix::getTick()}, AccelerometerData{x, y, z},
          TemperatureData{temp}
    {
    }
};

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
     *  @param chip_select  the chip_select for the sensor.
     *  @param _odr         output data rate for the accelerometer.
     *                      Default value is 100 Hz.
     *  @param _bdu         BlockDataUpdate value, continuous or non-continuous
     *                      update mode. Default value is to update after data 
     *                      has been read (BDU=1).
     *  @param _full_scale  full scale range (from +/-2g up to +/-16g).
     *                      Default value is +/-2g.
     */
    LIS3DSH(SPIBusInterface& bus, GpioPin chip_select,
            uint8_t _odr        = OutputDataRate::ODR_100_HZ,
            uint8_t _bdu        = BlockDataUpdate::UPDATE_AFTER_READ_MODE,
            uint8_t _full_scale = FullScale::FULL_SCALE_2G)
        : spi_slave(bus, chip_select, {}), odr(_odr), bdu(_bdu),
          full_scale(_full_scale)
    {
        spi_slave.config.clock_div =
            SPIClockDivider::DIV64;  // used to set the spi baud rate (maximum
                                     // is 10 Mhz)
    }

    /**
     *  @brief Constructor.
     *
     *  @param bus          the spi bus.
     *  @param chip_select  the chip_select for the sensor.
     *  @param config       the spi bus configurations.
     *  @param _odr         output data rate for the accelerometer.
     *                      Default value is 100 Hz.
     *  @param _bdu         BlockDataUpdate value, continuous or non-continuous
     *                      update mode. Default value is to update after data 
     *                      has been read (BDU=1).
     *  @param _full_scale  full scale range (from +/-2g up to +/-16g).
     *                      Default value is +/-2g.
     */
    LIS3DSH(SPIBusInterface& bus, GpioPin chip_select, SPIBusConfig config,
            uint8_t _odr        = OutputDataRate::ODR_100_HZ,
            uint8_t _bdu        = BlockDataUpdate::UPDATE_AFTER_READ_MODE,
            uint8_t _full_scale = FullScale::FULL_SCALE_2G)
        : spi_slave(bus, chip_select, config), odr(_odr), bdu(_bdu),
          full_scale(_full_scale)
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
            TRACE("[LIS3DSH] init() : already initialized \n");
            last_error = SensorErrors::ALREADY_INIT;
            return false;
        }

        // check if the sensor is working properly
        if (!checkWhoAmI())
        {                  // whoami default value
            return false;  // sensor correctly initialized
        }

        SPITransaction spi(spi_slave);

        // set the full scale value in CTRL_REG5
        uint8_t ctrl_reg5_value = (full_scale << 3);
        spi.write(CTRL_REG5, ctrl_reg5_value);

        // select the correct sensitivity
        // for the specified full scale range
        sensitivity = select_sensitivity();

        // set the output data rate and the BDU in CTRL_REG4
        // the three least significant bits are enable bits for X, Y and Z axis
        uint8_t ctrl_reg4_value =
            (odr << 4) | (bdu << 3) | (7 << 0);  // 7 = 111 -> enable the 3 axis
        spi.write(CTRL_REG4, ctrl_reg4_value);

        TRACE("[LIS3DSH] init() : ok \n");

        initialized = true;

        return true;
    }

    /**
     * @brief Read new data from the accelerometer.
     *        Acceleretions are returned in g.
     *
     * @return boolean value indicating whether the operation succeded or not
     */
    LIS3DSHData sampleImpl() override
    {
        // check if the sensor is initialized
        if (!initialized)
        {
            TRACE(
                "[LIS3DSH] sampleImpl() : not initialized, unable to "
                "sample data \n");
            last_error = SensorErrors::NOT_INIT;
            return data;
        }

        AccelerometerData acc_data = readAccelData();
        TemperatureData temp_data  = readTemperature();

        if (last_error != SensorErrors::NO_ERRORS)
        {
            return data;
        }
        else 
        {
            return LIS3DSHData{acc_data.accel_x, acc_data.accel_y, acc_data.accel_z, 
                                temp_data.temperature};
        }
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
            TRACE(
                "[LIS3DSH] selfTest() : not initialized, unable to self-test "
                "\n");
            last_error = SensorErrors::NOT_INIT;
            return false;
        }

        const uint8_t num_samples = 5;  // number of samples to be used
        // vectors for storing samples, both
        // in self-test and no-self-test modes
        float X_ST[num_samples]    = {0};
        float Y_ST[num_samples]    = {0};
        float Z_ST[num_samples]    = {0};
        float X_NO_ST[num_samples] = {0};
        float Y_NO_ST[num_samples] = {0};
        float Z_NO_ST[num_samples] = {0};
        // vectors containing avg values for each axis
        float AVG_ST[3]    = {0};  // one element per axis
        float AVG_NO_ST[3] = {0};  // one element per axis
        
        // set output data rate to 50 hz
        uint8_t ctrl_reg4_value = (OutputDataRate::ODR_100_HZ << 4) 
                        | (BlockDataUpdate::UPDATE_AFTER_READ_MODE << 3) 
                        | (7 << 0);

        {
            SPITransaction spi(spi_slave);
            spi.write(CTRL_REG4, ctrl_reg4_value);
        }

        // set full scale to default value +/-2g
        // enable the self-test mode with positive sign
        uint8_t ctrl_reg5_value = (FullScale::FULL_SCALE_2G << 3) | (1 << 1);

        {
            SPITransaction spi(spi_slave);
            spi.write(CTRL_REG5, ctrl_reg5_value);
        }

        // read samples in self-test positive sign mode
        for (uint8_t i = 0; i < num_samples; i++)
        {
            AccelerometerData acc_data = readAccelData();
            X_ST[i]                    = acc_data.accel_x;
            Y_ST[i]                    = acc_data.accel_y;
            Z_ST[i]                    = acc_data.accel_z;
            miosix::Thread::sleep(10);
        }
        // reset the self-test bits
        ctrl_reg5_value &= ~(3 << 1);
        // normal mode with full scale range +/-2g
        ctrl_reg5_value |= (FULL_SCALE_2G << 3);

        {
            SPITransaction spi(spi_slave);
            spi.write(CTRL_REG5, ctrl_reg5_value);
        }

        // read samples in normal mode
        for (uint8_t i = 0; i < num_samples; i++)
        {
            AccelerometerData acc_data = readAccelData();
            X_NO_ST[i]                 = acc_data.accel_x;
            Y_NO_ST[i]                 = acc_data.accel_y;
            Z_NO_ST[i]                 = acc_data.accel_z;
            miosix::Thread::sleep(10);
        }
        // compute averages vectors:
        // they contain one element for each axis
        // (position 0 for x, 1 for y and 2 for z)
        // AVG_ST    : for self-test samples
        // AVG_NO_ST : for normal mode samples
        for (uint8_t i = 0; i < num_samples; i++)
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
            AVG_ST[i] /= num_samples;
            AVG_NO_ST[i] /= num_samples;
        }

        // Reset registers values with the ones
        // specified in the constructor:
        // set the output data rate value in CTRL_REG4
        ctrl_reg4_value = (odr << 4) | (bdu << 3) | (7 << 0);

        {
            SPITransaction spi(spi_slave);
            spi.write(CTRL_REG4, ctrl_reg4_value);
        }
        // set the full scale value in CTRL_REG5
        ctrl_reg5_value = (full_scale << 3);  // normal mode

        {
            SPITransaction spi(spi_slave);
            spi.write(CTRL_REG5, ctrl_reg5_value);
        }

        float delta[3] = {0};
        for (uint8_t i = 0; i < 3; i++)
        {
            delta[i] = fabs(AVG_NO_ST[i] - AVG_ST[i]);
        }

        TRACE("[LIS3DSH] selfTest() : delta[x] = %f \n", delta[0]);
        TRACE("[LIS3DSH] selfTest() : delta[y] = %f \n", delta[1]);
        TRACE("[LIS3DSH] selfTest() : delta[z] = %f \n", delta[2]);

        // check that the averages differences
        // do not exceed maximum tolerance
        if ((delta[0] >
             SELF_TEST_DIFF_X_Y + SELF_TEST_DIFF_X_Y * SELF_TEST_TOLERANCE) ||
            (delta[1] >
             SELF_TEST_DIFF_X_Y + SELF_TEST_DIFF_X_Y * SELF_TEST_TOLERANCE) ||
            (delta[2] >
             SELF_TEST_DIFF_Z + SELF_TEST_DIFF_Z * SELF_TEST_TOLERANCE))
        {
            TRACE("[LIS3DSH] selfTest() : failed \n");
            last_error = SensorErrors::SELF_TEST_FAIL;
            return false;
        }

        TRACE("[LIS3DSH] selfTest() : ok \n");
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
     * @brief Read accelrometer data.
     *
     * @return the read accelrometer sample
     */
    AccelerometerData readAccelData()
    {
        AccelerometerData acc_data;

        SPITransaction spi(spi_slave);

        // read the sensor's status register
        uint8_t status = spi.read(STATUS);

        if (status & 0x08)
        {  // bit 3 of status set to 1 (new data available)
            if (status & 0x80)
            {  // bit 7 of status set to 1 (some data overwritten)

                // read acceleration on X
                int8_t acc_L = spi.read(OUT_X_L);
                int8_t acc_H = spi.read(OUT_X_H);
                acc_data.accel_x =
                    static_cast<float>(combine(acc_H, acc_L)) * sensitivity;

                // read acceleration on Y
                acc_L = spi.read(OUT_Y_L);
                acc_H = spi.read(OUT_Y_H);
                acc_data.accel_y =
                    static_cast<float>(combine(acc_H, acc_L)) * sensitivity;

                // read acceleration on Z
                acc_L = spi.read(OUT_Z_L);
                acc_H = spi.read(OUT_Z_H);
                acc_data.accel_z =
                    static_cast<float>(combine(acc_H, acc_L)) * sensitivity;

                last_error = SensorErrors::NO_ERRORS;
            }
        }
        else
        {
            last_error = SensorErrors::NO_NEW_DATA;
        }

        return acc_data;
    }

    /**
     * @brief Read temperature data.
     *
     * @return the read temperature sample
     */
    TemperatureData readTemperature()
    {
        SPITransaction spi(spi_slave);

        // the temperature is given as a 8-bits integer (in 2-complement)
        return TemperatureData{
            spi.read(OUT_T) +
            TEMPERATURE_REF};  // add the 'zero' of the temperature sensor
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
        SPITransaction spi(spi_slave);

        // check the WHO_AM_I_REG register
        uint8_t who_am_i_value = spi.read(WHO_AM_I_REG);
        if (who_am_i_value == WHO_AM_I_DEFAULT_VALUE)
        {  // whoami default value
            TRACE("[LIS3DSH] WHO_AM_I : ok \n");
            return true;
        }
        else
        {
            TRACE("[LIS3DSH] WHO_AM_I : wrong value, %d instead of %d \n",
                  who_am_i_value, WHO_AM_I_DEFAULT_VALUE);
            last_error = SensorErrors::INVALID_WHOAMI;
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
    float select_sensitivity()
    {
        float s;
        switch (full_scale)
        {
            case FULL_SCALE_2G:
                s = sensitivity_values[FullScale::FULL_SCALE_2G];
                break;
            case FULL_SCALE_4G:
                s = sensitivity_values[FullScale::FULL_SCALE_4G];
                break;
            case FULL_SCALE_6G:
                s = sensitivity_values[FullScale::FULL_SCALE_6G];
                break;
            case FULL_SCALE_8G:
                s = sensitivity_values[FullScale::FULL_SCALE_8G];
                break;
            case FULL_SCALE_16G:
                s = sensitivity_values[FullScale::FULL_SCALE_16G];
                break;
            default:
                TRACE(
                    "[LIS3DSH] Invalid full scale range given, using +/-2g \n");
                this->full_scale = FullScale::FULL_SCALE_2G;
                s                = sensitivity_values[FullScale::FULL_SCALE_2G];
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

    SPISlave spi_slave;

    bool initialized = false;  // whether the sensor has been initialized or not

    uint8_t odr;         // output data rate, default 100 Hz
    uint8_t bdu;         // continuous or block after update
    uint8_t full_scale;  // full scale value, default +/- 2g

    /**
     * @brief Sensitivity values corresponding to full scale range allowed
     * values.
     */
    const float sensitivity_values[5] = {0.06, 0.12, 0.18, 0.24, 0.73};

    float sensitivity =
        sensitivity_values[FullScale::FULL_SCALE_2G];  // default sensitivity
                                                       // value

    const uint8_t WHO_AM_I_DEFAULT_VALUE = 63;  // 00111111

    const float TEMPERATURE_REF = 25.0f;  // temperature sensor 'zero'/reference
                                          // value (value 0x00 from the sensor
                                          // corresponds to 25 degrees celsius)

    const float SELF_TEST_DIFF_X_Y  = 140.0f;  // 140 mg
    const float SELF_TEST_DIFF_Z    = 590.0f;  // 590 mg
    const float SELF_TEST_TOLERANCE = 0.3f;    // 10%
};
