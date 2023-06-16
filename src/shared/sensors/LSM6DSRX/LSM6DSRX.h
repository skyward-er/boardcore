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

#include "LSM6DSRXConfig.h"
#include "LSM6DSRXDefs.h"

namespace Boardcore
{

class LSM6DSRX
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
    bool init();

    /**
     * @brief Retrieves data from the accelerometer.
     * @param data The structure where data from the sensor is to be saved.
     */
    void getAccelerometerData(SensorData& data);

    /**
     * @brief Retrieves data from the gyroscope.
     * @param data The structure where data from the sensor is to be saved.
     */
    void getGyroscopeData(SensorData& data);

    /**
     * @brief Returns the timestamp from the sensor.
     */
    uint32_t getSensorTimestamp();

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
     * @return Returns number of batch red from the FIFO and stored in the
     * buffer.
     */
    int readFromFifo(FifoData buf[], int num)
    {
        uint8_t value        = 0;
        int numUnreadSamples = 0;
        SPITransaction spiTransaction{m_spiSlave};

        // reads the number of unread samples in the fifo
        numUnreadSamples = static_cast<int>(
            spiTransaction.readRegister(LSM6DSRXDefs::REG_FIFO_STATUS1));
        value = spiTransaction.readRegister(LSM6DSRXDefs::REG_FIFO_STATUS2);
        numUnreadSamples |= static_cast<int>(value & 3) << 8;

        if (numUnreadSamples < num)
        {
            // there is no enough data.
            num = numUnreadSamples;
        }

        for (int i = 0; i < num; ++i)
        {
            {
                SPITransaction spiTransaction{m_spiSlave};
                buf[i].tag = spiTransaction.readRegister(
                    LSM6DSRXDefs::REG_FIFO_DATA_OUT_TAG);
            }
            buf[i].x = getAxisData(LSM6DSRXDefs::REG_FIFO_DATA_OUT_X_L,
                                   LSM6DSRXDefs::REG_FIFO_DATA_OUT_X_H);
            buf[i].y = getAxisData(LSM6DSRXDefs::REG_FIFO_DATA_OUT_Y_L,
                                   LSM6DSRXDefs::REG_FIFO_DATA_OUT_Y_H);
            buf[i].z = getAxisData(LSM6DSRXDefs::REG_FIFO_DATA_OUT_Z_L,
                                   LSM6DSRXDefs::REG_FIFO_DATA_OUT_Z_H);
        }

        return num;
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
    bool selfTest();

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
};

}  // namespace Boardcore