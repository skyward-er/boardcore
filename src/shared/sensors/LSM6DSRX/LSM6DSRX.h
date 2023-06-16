/* Copyright (c) 2022 Skyward Experimental Rocketry
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

#include <drivers/spi/SPIDriver.h>
#include <miosix.h>
#include <sensors/Sensor.h>

#include "LSM6DSRXConfig.h"
#include "LSM6DSRXData.h"
#include "LSM6DSRXDefs.h"

namespace Boardcore
{

class LSM6DSRX : SensorFIFO<LSM6DSRXData, 50>
{
public:
    /**
     * @brief Struct used to store data from the accelerometer and gyroscope.
     */
    struct SensorData
    {
        float x;
        float y;
        float z;
    };

    /**
     * @brief Struct used to store data from FIFO.
     */
    struct FifoData
    {
        uint16_t x;  ///< value red from REG_FIFO_DATA_OUT_X
        uint16_t y;
        uint16_t z;
        uint8_t tag;  ///< value red from REG_FIFO_DATA_OUT_TAG
    };

    /**
     * @brief LSM6DSRX constructor.
     *
     * @param bus SPI bus.
     * @param csPin SPI chip select pin.
     * @param busConfiguration SPI bus configuration.
     * @param config LSM6DSRX configuration.
     */
    LSM6DSRX(SPIBus& bus, miosix::GpioPin csPin, SPIBusConfig busConfiguration,
             LSM6DSRXConfig& config);

    /**
     * @brief Initialize the sensor.
     */
    bool init() override;

    /**
     * @brief Returns the timestamp resolution in milliseconds.
     * USED ONLY FOR TESTING, TO BE REMOVED
     */
    float getSensorTimestampResolution()
    {
        SPITransaction spi{m_spiSlave};

        uint8_t value = spi.readRegister(LSM6DSRXDefs::REG_INTERNAL_FREQ_FINE);

        // TS_Res = 1 / (40000 + (0.0015 * INTERNAL_FREQ_FINE * 40000))
        float TS_Res = 1 / (40000 + (0.0015 * value * 40000));
        return TS_Res * 1000;
    }

    /**
     * @brief Performs a really simple reading from the FIFO buffer.
     * @param buf Buffer where to save data.
     * @param num Number of batch to be red from the FIFO.
     */
    void readFromFifo()
    {
        uint8_t value        = 0;
        int numUnreadSamples = 0;
        SPITransaction spi{m_spiSlave};
        int num = lastFifo.max_size();  // number of sample to read
        int i   = 0;

        // when extracting data from fifo i get data only from one sensor, but
        // the struct LSM6DSRXData is made to have data from both sensors. The
        // idea is to copy the value from the last valid sample for the sensor
        // that hasn't received data.
        LSM6DSRXData lastValidSample;
        lastValidSample.accelerationTimestamp = 0;
        lastValidSample.angularSpeedTimestamp = 0;
        lastValidSample.accelerationX         = 0.0;
        lastValidSample.accelerationY         = 0.0;
        lastValidSample.accelerationZ         = 0.0;
        lastValidSample.angularSpeedX         = 0.0;
        lastValidSample.angularSpeedY         = 0.0;
        lastValidSample.angularSpeedZ         = 0.0;

        // data has 2bits tags that determins the corresponding time slot.
        // 00 -> first element of the array (timestamps[0])
        // 11 -> last element of the array (timestamps[3])
        // when a new timestamp is received from the fifo the array is updated.
        // NOTE: those timestamps are already converted from sensor ones to
        // TimestampTimer class.
        uint64_t timestamps[4] = {0};

        // reads the number of unread samples in the fifo
        numUnreadSamples =
            static_cast<int>(spi.readRegister(LSM6DSRXDefs::REG_FIFO_STATUS1));
        value = spi.readRegister(LSM6DSRXDefs::REG_FIFO_STATUS2);
        numUnreadSamples |= static_cast<int>(value & 3) << 8;

        if (numUnreadSamples < num)
        {
            // there is no enough data.
            num = numUnreadSamples;
        }

        for (i = 0; i < num; ++i)
        {
            const uint8_t sampleTag =
                spi.readRegister(LSM6DSRXDefs::REG_FIFO_DATA_OUT_TAG);
            const uint8_t sensorTag   = (sampleTag >> 3) & 31;
            const uint8_t timeslotTag = sampleTag & 6;  // & 0b0110

            switch (sensorTag)
            {
                case 0x01:
                    // gyroscope data
                    lastFifo[i].angularSpeedX = getAxisData(
                        LSM6DSRXDefs::REG_FIFO_DATA_OUT_X_L,
                        LSM6DSRXDefs::REG_FIFO_DATA_OUT_X_H, m_sensitivityGyr);
                    lastFifo[i].angularSpeedY = getAxisData(
                        LSM6DSRXDefs::REG_FIFO_DATA_OUT_Y_L,
                        LSM6DSRXDefs::REG_FIFO_DATA_OUT_Y_H, m_sensitivityGyr);
                    lastFifo[i].angularSpeedZ = getAxisData(
                        LSM6DSRXDefs::REG_FIFO_DATA_OUT_Z_L,
                        LSM6DSRXDefs::REG_FIFO_DATA_OUT_Z_H, m_sensitivityGyr);

                    // TODO: COME PRENDO TIMESTAMP?
                    lastFifo[i].angularSpeedTimestamp = timestamps[timeslotTag];

                    // CHECK: va bene cosi'? o devo vedere quali valori hanno lo
                    // stesso timestamp e metterli insieme?
                    lastFifo[i].accelerationX = lastValidSample.accelerationX;
                    lastFifo[i].accelerationY = lastValidSample.accelerationY;
                    lastFifo[i].accelerationZ = lastValidSample.accelerationZ;
                    lastFifo[i].accelerationTimestamp =
                        lastValidSample.accelerationTimestamp;

                    // update last valid sample for the gyroscope
                    lastValidSample.angularSpeedTimestamp =
                        lastFifo[i].angularSpeedTimestamp;
                    lastValidSample.angularSpeedX = lastFifo[i].angularSpeedX;
                    lastValidSample.angularSpeedY = lastFifo[i].angularSpeedY;
                    lastValidSample.angularSpeedZ = lastFifo[i].angularSpeedZ;

                    break;
                case 0x02:
                    // accelerometer data
                    lastFifo[i].accelerationX = getAxisData(
                        LSM6DSRXDefs::REG_FIFO_DATA_OUT_X_L,
                        LSM6DSRXDefs::REG_FIFO_DATA_OUT_X_H, m_sensitivityAcc);
                    lastFifo[i].accelerationY = getAxisData(
                        LSM6DSRXDefs::REG_FIFO_DATA_OUT_Y_L,
                        LSM6DSRXDefs::REG_FIFO_DATA_OUT_Y_H, m_sensitivityAcc);
                    lastFifo[i].accelerationZ = getAxisData(
                        LSM6DSRXDefs::REG_FIFO_DATA_OUT_Z_L,
                        LSM6DSRXDefs::REG_FIFO_DATA_OUT_Z_H, m_sensitivityAcc);

                    // TODO: COME PRENDO TIMESTAMP?
                    lastFifo[i].accelerationTimestamp = timestamps[timeslotTag];

                    // CHECK: va bene cosi'? o devo vedere quali valori hanno lo
                    // stesso timestamp e metterli insieme?
                    lastFifo[i].angularSpeedX = lastValidSample.angularSpeedX;
                    lastFifo[i].angularSpeedY = lastValidSample.angularSpeedY;
                    lastFifo[i].angularSpeedZ = lastValidSample.angularSpeedZ;
                    lastFifo[i].angularSpeedTimestamp =
                        lastValidSample.angularSpeedTimestamp;

                    // update lastValidSampe for the accelerometer
                    lastValidSample.accelerationTimestamp =
                        lastFifo[i].accelerationTimestamp;
                    lastValidSample.accelerationX = lastFifo[i].accelerationX;
                    lastValidSample.accelerationY = lastFifo[i].accelerationY;
                    lastValidSample.accelerationZ = lastFifo[i].accelerationZ;

                    break;
                case 0x03:
                    // temperature data

                    // NECESSARIO? VERRA' CAMPIONATA? perche' se si devo
                    // aggiungere il valore a LSM6DSRXData

                    break;
                case 0x04:
                    // timestamp data --> update timestamps

                    // assemblo dati
                    uint64_t t = static_cast<uint64_t>(
                        getAxisData(LSM6DSRXDefs::REG_FIFO_DATA_OUT_X_L,
                                    LSM6DSRXDefs::REG_FIFO_DATA_OUT_X_H));
                    t |= static_cast<uint64_t>(
                             getAxisData(LSM6DSRXDefs::REG_FIFO_DATA_OUT_Y_L,
                                         LSM6DSRXDefs::REG_FIFO_DATA_OUT_Y_H))
                         << 16;

                    timestamps[timeslotTag] = convertTimestamp(t);

                    break;
                    // default:
                    //     // last_error_bad_data;
                    //     break;
            }
        }

        // CHECK
        lastFifoLevel = num;  // corretto??
    }

    /**
     * @brief Returns the number of unread data in the fifo.
     */
    unsigned int unreadDataInFifo()
    {
        unsigned int ris = 0;
        SPITransaction spi{m_spiSlave};

        ris = spi.readRegister(LSM6DSRXDefs::REG_FIFO_STATUS1);
        ris = ris | (static_cast<unsigned int>(
                         spi.readRegister(LSM6DSRXDefs::REG_FIFO_STATUS2) & 3)
                     << 8);
        return ris;
    }

    /**
     * @brief Performs self test for the sensor.
     * @return Return true if the test was successful.
     */
    bool selfTest() override;

    /**
     * @brief Returns data from the sensors.
     */
    LSM6DSRXData getSensorData();

    /**
     * @brief Gather data from FIFO/data registers and temperature sensor.
     */
    LSM6DSRXData sampleImpl() override;

private:
    bool m_isInit = false;
    SPISlave m_spiSlave;
    LSM6DSRXConfig m_config;

    float m_sensitivityAcc = 0.0;  ///< Sensitivity value for the accelerator.
    float m_sensitivityGyr = 0.0;  ///< Sensitivity value for the gyroscope.

    /**
     * @brief Check who_am_i register for validity.
     *
     * @return Returns false if not valid.
     */
    bool checkWhoAmI();

    /**
     * @brief Utility for combining two 8 bits numbers in one 16 bits number.
     * @param low Low bits of the 16 bit number.
     * @param high High bits of the 16 bit number.
     */
    int16_t combineHighLowBits(uint8_t low, uint8_t high);

    /**
     * @brief Reads 16-bits float data from the specified registers.
     * @param lowReg Register containing the low bits of the output.
     * @param highReg Register containing the high bits of the output.
     * @param sensitivity Sensitivity value for the sample.
     */
    float getAxisData(LSM6DSRXDefs::Registers lowReg,
                      LSM6DSRXDefs::Registers highReg, float sensitivity);

    /**
     * @brief Reads 16-bits data from the specified registers.
     * @param lowReg Register containing the low bits of the output.
     * @param highReg Register containing the high bits of the output.
     */
    int16_t getAxisData(LSM6DSRXDefs::Registers lowReg,
                        LSM6DSRXDefs::Registers highReg)
    {
        int8_t low = 0, high = 0;
        int16_t sample = 0;

        SPITransaction transaction{m_spiSlave};

        high = transaction.readRegister(highReg);
        low  = transaction.readRegister(lowReg);

        sample = combineHighLowBits(low, high);

        return sample;
    }

    /**
     * @brief Initialize the accelerometer.
     */
    bool initAccelerometer();

    /**
     * @brief Initialize the gyroscope.
     */
    bool initGyroscope();

    /**
     * @brief Initialize fifo.
     */
    bool initFifo();

    /**
     * @brief Initialize interrupts.
     */
    void initInterrupts();

    /**
     * @brief Performs self test for the accelerometer.
     * @return Return true if successful.
     */
    bool selfTestAcc();

    /**
     * @brief Performs self test for the gyroscope.
     * @return Return true if successful.
     */
    bool selfTestGyr();

    /**
     * @brief Retrieves data from the accelerometer.
     * @param data The structure where data from the sensor is to be saved.
     */
    void getAccelerometerData(LSM6DSRXData& data);

    /**
     * @brief Retrieves data from the gyroscope.
     * @param data The structure where data from the sensor is to be saved.
     */
    void getGyroscopeData(LSM6DSRXData& data);

    /**
     * @brief Returns the timestamp from the sensor.
     */
    uint32_t getSensorTimestamp();

    /**
     * @brief Converts timestamp from the value given by the sensor to the one
     * given by TimestampTimer class.
     * @param t The timestamp to be converted.
     */
    uint64_t convertTimestamp(uint64_t t);
};

}  // namespace Boardcore
